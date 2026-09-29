"""
main_controller.py

A starter controller with basic obstacle avoidance.
"""

from controller import Robot

TIME_STEP = 64
MAX_SPEED = 6.0

robot = Robot()

# 1. Initialize Motors
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 2. Initialize LEDs
led_blue = robot.getDevice('led_blue')
led_green = robot.getDevice('led_green')
led_blue.set(0)
led_green.set(0)

# 3. Initialize Sensors
ds_front = robot.getDevice('ds_front')
ds_left = robot.getDevice('ds_left')
ds_right = robot.getDevice('ds_right')

ds_front.enable(TIME_STEP)
ds_left.enable(TIME_STEP)
ds_right.enable(TIME_STEP)

# (Optional) Initialize Camera and Inertial Unit for later
camera = robot.getDevice('camera')
if camera:
    camera.enable(TIME_STEP)

inertial_unit = robot.getDevice('inertial_unit')
if inertial_unit:
    inertial_unit.enable(TIME_STEP)

print("Starting Basic Obstacle Avoidance...")

# 4. Main Control Loop
while robot.step(TIME_STEP) != -1:
    
    # Read sensor values
    front_val = ds_front.getValue()
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()

    # The default DistanceSensor returns 1000 when an object is touching it, 
    # and drops towards 0 the further away the object is.
    # Let's say anything above 80 means an obstacle is too close.
    obstacle_front = front_val > 80
    obstacle_left = left_val > 80
    obstacle_right = right_val > 80

    # Default action is to drive straight
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # Basic Avoidance Logic:
    if obstacle_front:
        # If there's a wall in front, turn! 
        # We can check which side has more space to decide which way to turn.
        if left_val > right_val:
            # Left wall is closer, so turn right
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
        else:
            # Right wall is closer, so turn left
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
            
    elif obstacle_left:
        # Too close to left wall, veer right slightly
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED * 0.5
        
    elif obstacle_right:
        # Too close to right wall, veer left slightly
        left_speed = MAX_SPEED * 0.5
        right_speed = MAX_SPEED

    # Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
