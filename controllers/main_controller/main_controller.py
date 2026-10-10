"""
main_controller.py - Right Wall Follower & Red Tile Counter
"""

from controller import Robot
import math

TIME_STEP = 16
MAX_SPEED = 6.0
SENSOR_NAMES = (
    [f"ds_front_{i}" for i in range(5)] +
    [f"ds_back_{i}" for i in range(5)] +
    [f"ds_left_{i}" for i in range(5)] +
    [f"ds_right_{i}" for i in range(5)]
)

def main():
    robot = Robot()
    
    # 1. Initialize Motors
    left_motor = robot.getDevice('left_wheel_motor')
    right_motor = robot.getDevice('right_wheel_motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # 2. Initialize Distance Sensors
    ds = {}
    for name in SENSOR_NAMES:
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        ds[name] = sensor
        
    # 3. Initialize Camera
    camera = robot.getDevice('camera')
    camera.enable(TIME_STEP)
    cam_width = camera.getWidth()
    cam_height = camera.getHeight()
    
    # 4. Initialize LEDs
    led_blue = robot.getDevice('led_blue')
    led_green = robot.getDevice('led_green')
    led_blue.set(0)
    led_green.set(0)
    
    # State Variables
    red_tile_count = 0
    currently_on_red = False
    
    print("--- STARTING AUTONOMOUS RUN ---")
    
    while robot.step(TIME_STEP) != -1:
        # Read all 20 sensors
        dists = {}
        for name in SENSOR_NAMES:
            d = ds[name].getValue()
            if d != d: d = 0.6
            dists[name] = d
            
        # Get minimum distance on the front bumper
        front_dist = min([dists[f"ds_front_{i}"] for i in range(5)])
        
        # Get minimum distance on the right bumper
        right_dist = min([dists[f"ds_right_{i}"] for i in range(5)])
        
        # Check Camera for Red Tiles
        img = camera.getImageArray()
        if img:
            r, g, b = img[cam_width // 2][cam_height // 2]
            
            # Simple threshold for strong Red
            is_red = (r > 120 and r > g * 1.5 and r > b * 1.5)
            
            if is_red and not currently_on_red:
                currently_on_red = True
                red_tile_count += 1
                
                if red_tile_count == 1:
                    print("[TILE 1] Detected STARTING CELL.")
                elif red_tile_count == 2:
                    print("[TILE 2] Detected TRANSITION CELL! Entering Section 2.")
                elif red_tile_count == 3:
                    print("[TILE 3] Detected FINAL END CELL! Stopping.")
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    led_blue.set(1) # Light up an LED to finish
                    break
            elif not is_red:
                currently_on_red = False
                
        # --- ROBUST RIGHT WALL FOLLOWING ---
        
        # 1. Frontal Collision Avoidance
        if front_dist < 0.10:
            # Obstacle directly in our path! Spin left in place to clear it.
            v_l = -MAX_SPEED * 0.8
            v_r = MAX_SPEED * 0.8
        else:
            # 2. Constant Distance Wall Following (P Controller)
            target_dist = 0.08  # Stay 8cm from the right wall in tight corridors
            
            if right_dist > 0.20:
                # Lost the wall entirely. Curve right to find it again.
                v_l = MAX_SPEED * 0.8
                v_r = MAX_SPEED * 0.3
            else:
                # Wall is in sight, apply Proportional control to stay exactly at target_dist
                error = target_dist - right_dist
                Kp = 18.0
                
                turn_rate = Kp * error
                
                base_speed = MAX_SPEED * 0.8
                v_l = base_speed - turn_rate
                v_r = base_speed + turn_rate
                
                # Clamp speeds to motor limits
                v_l = max(-MAX_SPEED, min(MAX_SPEED, v_l))
                v_r = max(-MAX_SPEED, min(MAX_SPEED, v_r))
                
        left_motor.setVelocity(v_l)
        right_motor.setVelocity(v_r)

if __name__ == "__main__":
    main()
