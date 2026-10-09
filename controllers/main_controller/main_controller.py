"""
main_controller.py - Right Wall Follower & Red Tile Counter
"""

from controller import Robot
import math

TIME_STEP = 16
MAX_SPEED = 6.0

# 8 Sensors (Name, Angle_in_degrees)
SENSORS = [
    ("ds_front", 0.0),
    ("ds_front_left", 45.0),
    ("ds_left", 90.0),
    ("ds_back_left", 135.0),
    ("ds_back", 180.0),
    ("ds_back_right", -135.0),
    ("ds_right", -90.0),
    ("ds_front_right", -45.0)
]

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
    for name, _ in SENSORS:
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
        # Read sensors
        front_dist = ds['ds_front'].getValue()
        front_right_dist = ds['ds_front_right'].getValue()
        right_dist = ds['ds_right'].getValue()
        
        # Replace NaN with max range (0.8)
        if front_dist != front_dist: front_dist = 0.8
        if front_right_dist != front_right_dist: front_right_dist = 0.8
        if right_dist != right_dist: right_dist = 0.8
        
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
                
        # Right Wall Following Logic
        v_l = MAX_SPEED
        v_r = MAX_SPEED
        
        if front_dist < 0.25:
            # Wall in front -> turn left sharply
            v_l = -MAX_SPEED * 0.5
            v_r = MAX_SPEED * 0.5
        else:
            if right_dist > 0.35:
                # Too far from right wall -> turn right to hug it
                v_l = MAX_SPEED * 0.7
                v_r = MAX_SPEED * 0.3
            elif right_dist < 0.15 or front_right_dist < 0.15:
                # Too close to right wall -> turn left to avoid scraping
                v_l = MAX_SPEED * 0.3
                v_r = MAX_SPEED * 0.7
            else:
                # Good distance, go straight
                v_l = MAX_SPEED
                v_r = MAX_SPEED
                
        left_motor.setVelocity(v_l)
        right_motor.setVelocity(v_r)

if __name__ == "__main__":
    main()
