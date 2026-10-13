"""
controller_section1.py

Section 1: Autonomous Obstacle Navigation
Uses Reinforcement Learning (RLPolicy) for reactive, collision-free navigation
from the starting zone to the Section 2 transition gate.
"""

import math
import os
import sys

try:
    from rl_policy import RLPolicy
except ImportError:
    from controllers.main_controller.rl_policy import RLPolicy

TIME_STEP = 16
MAX_WHEEL = 10.0
WHEEL_R = 0.02
TRANSITION_GOAL = (0.13, 1.29)  # Coordinates of transition entrance
STARTING_POS = (2.26, 0.16)

SENSORS = [
    ("ds_front", 0.042, 0, 0.0),
    ("ds_front_left", 0.0297, 0.0297, 45.0),
    ("ds_left", 0, 0.042, 90.0),
    ("ds_back_left", -0.0297, 0.0297, 135.0),
    ("ds_back", -0.042, 0, 180.0),
    ("ds_back_right", -0.0297, -0.0297, -135.0),
    ("ds_right", 0, -0.042, -90.0),
    ("ds_front_right", 0.0297, -0.0297, -45.0)
]

def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class Section1Controller:
    def __init__(self, robot, goal=TRANSITION_GOAL):
        self.robot = robot
        self.goal = goal
        
        # Motors
        self.lm = robot.getDevice("left_wheel_motor")
        self.rm = robot.getDevice("right_wheel_motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        self.lm.setVelocity(0.0)
        self.rm.setVelocity(0.0)
        
        # Encoders & IMU
        self.ls = robot.getDevice("left_wheel_sensor")
        self.rs = robot.getDevice("right_wheel_sensor")
        self.ls.enable(TIME_STEP)
        self.rs.enable(TIME_STEP)
        
        self.imu = robot.getDevice("inertial_unit")
        self.imu.enable(TIME_STEP)
        
        # Distance Sensors
        self.ds_devices = []
        for name, _, _, _ in SENSORS:
            dev = robot.getDevice(name)
            dev.enable(TIME_STEP)
            self.ds_devices.append(dev)
            
        self.d = [0.8] * 8
        self.x, self.y = STARTING_POS
        self.th = math.pi  # Facing West towards the goal initially
        self.wl_prev = None
        self.wr_prev = None
        
        # RL Policy
        self.policy = RLPolicy()
        self.step_count = 0
        self.finished = False

    def update_odometry(self):
        """Update dead-reckoning position from wheel encoders and IMU heading."""
        self.th = self.imu.getRollPitchYaw()[2]
        wl = self.ls.getValue()
        wr = self.rs.getValue()
        
        if self.wl_prev is not None and self.wr_prev is not None:
            ds = 0.5 * ((wl - self.wl_prev) + (wr - self.wr_prev)) * WHEEL_R
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
            
        self.wl_prev, self.wr_prev = wl, wr

    def update_sensors(self):
        """Read all 8 distance sensors and clamp to [0.0, 0.8]m."""
        for k, dev in enumerate(self.ds_devices):
            val = dev.getValue()
            self.d[k] = 0.8 if val != val else max(0.0, min(0.8, val))

    def get_observation(self):
        """
        Constructs normalized 11-dim observation vector for the RL policy:
        [d0/0.8, ..., d7/0.8, dist/3.0, sin(heading_error), cos(heading_error)]
        """
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        dist = math.hypot(dx, dy)
        goal_heading = math.atan2(dy, dx)
        heading_err = wrap_angle(goal_heading - self.th)
        
        norm_dists = [dist_val / 0.8 for dist_val in self.d]
        norm_goal_dist = min(dist / 3.0, 1.0)
        sin_err = math.sin(heading_err)
        cos_err = math.cos(heading_err)
        
        obs = norm_dists + [norm_goal_dist, sin_err, cos_err]
        return obs, dist

    def step(self):
        """
        Executes one control cycle for Section 1.
        Returns True if Section 1 is ongoing, False if goal reached.
        """
        self.update_sensors()
        self.update_odometry()
        
        obs, dist_to_goal = self.get_observation()
        
        # Check transition gate arrival
        if dist_to_goal < 0.08:
            print(f"[SECTION 1] Reached Transition Zone at ({self.x:.3f}, {self.y:.3f})! Dist={dist_to_goal:.3f}m")
            self.lm.setVelocity(0.0)
            self.rm.setVelocity(0.0)
            self.finished = True
            return False
            
        # RL Policy Action
        act_l, act_r = self.policy.predict(obs)
        v_l = max(-MAX_WHEEL, min(MAX_WHEEL, act_l * MAX_WHEEL))
        v_r = max(-MAX_WHEEL, min(MAX_WHEEL, act_r * MAX_WHEEL))
        
        # Safety Reflex: Prevent collisions with side corners
        if self.d[1] < 0.07:  # Front-left too close
            v_l += 2.0
            v_r -= 2.0
        elif self.d[7] < 0.07:  # Front-right too close
            v_l -= 2.0
            v_r += 2.0
            
        self.lm.setVelocity(v_l)
        self.rm.setVelocity(v_r)
        
        self.step_count += 1
        if self.step_count % 100 == 0:
            print(f"[SECTION 1] Step {self.step_count}: Pos=({self.x:.2f}, {self.y:.2f}), DistToGoal={dist_to_goal:.2f}m")
            
        return True

    def is_finished(self):
        return self.finished
