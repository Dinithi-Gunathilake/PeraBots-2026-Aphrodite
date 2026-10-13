"""
webots_env.py

Gymnasium-compatible Webots Simulation Environment for Section 1 RL Training.
Uses Webots Supervisor capabilities during training to reset robot pose,
randomize positions, and compute ground-truth distances and rewards.
"""

import math
import numpy as np

try:
    from controller import Supervisor
except ImportError:
    Supervisor = None

class WebotsSection1Env:
    def __init__(self, goal=(0.13, 1.29), max_steps=800):
        if Supervisor is None:
            raise RuntimeError("Webots controller module not found. Run inside Webots or set WEBOTS_HOME/lib/controller/python in PYTHONPATH.")
            
        self.supervisor = Supervisor()
        self.time_step = int(self.supervisor.getBasicTimeStep())
        self.goal = np.array(goal, dtype=np.float32)
        self.max_steps = max_steps
        self.current_step = 0
        
        # Actuators
        self.lm = self.supervisor.getDevice("left_wheel_motor")
        self.rm = self.supervisor.getDevice("right_wheel_motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        self.lm.setVelocity(0.0)
        self.rm.setVelocity(0.0)
        
        # IMU & Distance Sensors
        self.imu = self.supervisor.getDevice("inertial_unit")
        self.imu.enable(self.time_step)
        
        self.sensor_names = [
            "ds_front", "ds_front_left", "ds_left", "ds_back_left",
            "ds_back", "ds_back_right", "ds_right", "ds_front_right"
        ]
        self.sensors = [self.supervisor.getDevice(name) for name in self.sensor_names]
        for s in self.sensors:
            s.enable(self.time_step)
            
        # Supervisor Node handles for resetting
        self.robot_node = self.supervisor.getSelf()
        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")
        
        self.prev_dist = 0.0

    def reset(self):
        """Resets the robot to starting zone with slight noise."""
        self.current_step = 0
        self.lm.setVelocity(0.0)
        self.rm.setVelocity(0.0)
        
        # Starting zone: (2.26, 0.16) +/- 0.04m, facing West (+/- 0.5 rad)
        rx = 2.26 + np.random.uniform(-0.04, 0.04)
        ry = 0.16 + np.random.uniform(-0.04, 0.04)
        r_angle = math.pi + np.random.uniform(-0.5, 0.5)
        
        self.trans_field.setSFVec3f([rx, ry, 0.005])
        self.rot_field.setSFRotation([0, 0, 1, r_angle])
        self.robot_node.resetPhysics()
        
        # Step simulation to settle sensors
        for _ in range(5):
            self.supervisor.step(self.time_step)
            
        pos = np.array(self.trans_field.getSFVec3f()[:2], dtype=np.float32)
        self.prev_dist = np.linalg.norm(self.goal - pos)
        
        return self._get_observation()

    def _get_observation(self):
        pos = np.array(self.trans_field.getSFVec3f()[:2], dtype=np.float32)
        yaw = self.imu.getRollPitchYaw()[2]
        
        # 8 distance sensors normalized to [0, 1]
        norm_dists = [min(0.8, max(0.0, s.getValue())) / 0.8 for s in self.sensors]
        
        # Relative goal vector
        rel = self.goal - pos
        dist = np.linalg.norm(rel)
        goal_ang = math.atan2(rel[1], rel[0])
        heading_err = (goal_ang - yaw + math.pi) % (2.0 * math.pi) - math.pi
        
        norm_dist = min(dist / 3.0, 1.0)
        sin_err = math.sin(heading_err)
        cos_err = math.cos(heading_err)
        
        obs = np.array(norm_dists + [norm_dist, sin_err, cos_err], dtype=np.float32)
        return obs, dist

    def step(self, action):
        """
        action: tuple (v_left_norm, v_right_norm) in [-1.0, 1.0]
        """
        self.current_step += 1
        
        # Apply actions (scale to max wheel speed 10.0 rad/s)
        v_l = float(np.clip(action[0], -1.0, 1.0)) * 10.0
        v_r = float(np.clip(action[1], -1.0, 1.0)) * 10.0
        self.lm.setVelocity(v_l)
        self.rm.setVelocity(v_r)
        
        if self.supervisor.step(self.time_step) == -1:
            obs, _ = self._get_observation()
            return obs, 0.0, True, {"reason": "simulation_ended"}
            
        obs, curr_dist = self._get_observation()
        
        # Reward components
        progress = (self.prev_dist - curr_dist) * 20.0
        self.prev_dist = curr_dist
        step_penalty = -0.02
        reward = progress + step_penalty
        
        min_sensor = min(obs[:8]) * 0.8  # In meters
        terminated = False
        info = {}
        
        # Collision detection (chassis radius ~ 0.045m)
        if min_sensor < 0.035:
            reward -= 50.0
            terminated = True
            info["reason"] = "collision"
        # Transition zone reached
        elif curr_dist < 0.08:
            reward += 100.0
            terminated = True
            info["reason"] = "goal_reached"
        # Timeout
        elif self.current_step >= self.max_steps:
            terminated = True
            info["reason"] = "timeout"
            
        return obs, reward, terminated, info
