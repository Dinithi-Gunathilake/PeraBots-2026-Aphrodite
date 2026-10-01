"""
main_controller.py

Basic obstacle avoidance and maze navigation logic.
"""

from controller import Robot

TIME_STEP = 16
MAX_SPEED = 6.0

robot = Robot()

# 1. Initialize Motors
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

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

camera = robot.getDevice('camera')
if camera:
    camera.enable(TIME_STEP)

inertial_unit = robot.getDevice('inertial_unit')
if inertial_unit:
    inertial_unit.enable(TIME_STEP)

# Step once to initialize the sensors before reading them
robot.step(TIME_STEP)

# Capture the direction the robot is facing at the start of the simulation
TARGET_YAW = inertial_unit.getRollPitchYaw()[2]

print(f"Target Heading Locked: {TARGET_YAW:.2f} radians")
print("Starting Target Heading + Obstacle Avoidance...")

# 4. Main Control Loop
while robot.step(TIME_STEP) != -1:
    
    # Read distance sensor values
    front_val = ds_front.getValue()
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()
    
    # Read current orientation
    current_yaw = inertial_unit.getRollPitchYaw()[2]

    # Calculate difference between current heading and target heading
    # We want to keep this difference as close to 0 as possible
    angle_diff = current_yaw - TARGET_YAW
    
    # Normalize the angle difference to handle the -pi to pi boundary wrap-around
    if angle_diff > 3.14159:
        angle_diff -= 6.28318
    elif angle_diff < -3.14159:
        angle_diff += 6.28318

    # The generic DistanceSensor returns 1000 when the path is CLEAR
    obstacle_front = front_val < 900
    obstacle_left = left_val < 900
    obstacle_right = right_val < 900

    # Default action is to drive straight
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    # ---------------------------------------------
    # 1. OBSTACLE AVOIDANCE (Highest Priority)
    # ---------------------------------------------
    if obstacle_front:
        # If there's a wall in front, check which side has more space
        if left_val > right_val:
            # Left wall is closer, so turn right sharply
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
        else:
            # Right wall is closer, so turn left sharply
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
            
    elif obstacle_left:
        # Too close to left wall, veer right sharply
        left_speed = MAX_SPEED
        right_speed = 0.0
        
    elif obstacle_right:
        # Too close to right wall, veer left sharply
        left_speed = 0.0
        right_speed = MAX_SPEED

    # ---------------------------------------------
    # 2. TARGET HEADING (Run only if path is clear)
    # ---------------------------------------------
    else:
        # If angle_diff is positive, the robot is facing too far left -> turn right
        if angle_diff > 0.1:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED * 0.5
        # If angle_diff is negative, the robot is facing too far right -> turn left
        elif angle_diff < -0.1:
            left_speed = MAX_SPEED * 0.5
            right_speed = MAX_SPEED
        else:
            # We are facing the correct direction! Go straight.
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

    # Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
