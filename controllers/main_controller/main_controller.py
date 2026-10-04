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
ds_front_left = robot.getDevice('ds_front_left')
ds_front_right = robot.getDevice('ds_front_right')
ds_left = robot.getDevice('ds_left')
ds_right = robot.getDevice('ds_right')

ds_front.enable(TIME_STEP)
ds_front_left.enable(TIME_STEP)
ds_front_right.enable(TIME_STEP)
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
print("Starting Target Heading + Obstacle Avoidance with Escape Maneuvers...")

# State Variables for the Escape Maneuver
stuck_counter = 0
escape_counter = 0
is_escaping = False
loop_counter = 0

# 4. Main Control Loop
while robot.step(TIME_STEP) != -1:
    
    # Read distance sensor values
    front_val = ds_front.getValue()
    fl_val = ds_front_left.getValue()
    fr_val = ds_front_right.getValue()
    left_val = ds_left.getValue()
    right_val = ds_right.getValue()
    
    # Read current orientation
    current_yaw = inertial_unit.getRollPitchYaw()[2]

    # Calculate difference between current heading and target heading
    angle_diff = current_yaw - TARGET_YAW
    
    # Normalize the angle difference to handle the -pi to pi boundary wrap-around
    if angle_diff > 3.14159:
        angle_diff -= 6.28318
    elif angle_diff < -3.14159:
        angle_diff += 6.28318

    # The generic DistanceSensor returns 1000 when the path is CLEAR
    obstacle_front = front_val < 900 or fl_val < 400 or fr_val < 400
    obstacle_left = left_val < 900
    obstacle_right = right_val < 900

    loop_counter += 1
    if loop_counter % 60 == 0:
        print(f"--- 1 Second Heartbeat | current stuck_counter: {stuck_counter} ---")

    # ---------------------------------------------
    # ESCAPE MANEUVER LOGIC
    # ---------------------------------------------
    if not is_escaping:
        # If we detect ANY obstacle, increase the stuck counter
        if obstacle_front or obstacle_left or obstacle_right:
            stuck_counter += 1
        else:
            stuck_counter = 0 # Path is clear, reset counter
            
        # If we've been struggling with an obstacle for ~2 seconds (120 steps at 16ms)
        if stuck_counter > 100:
            print(f"Robot is stuck! Readings -> Front: {front_val:.1f}, FL: {fl_val:.1f}, FR: {fr_val:.1f}, Left: {left_val:.1f}, Right: {right_val:.1f}")
            print("Initiating Escape Maneuver!")
            is_escaping = True
            escape_counter = 90  # 90 steps total for the escape
            stuck_counter = 0

    if is_escaping:
        # Print readings during the escape maneuver so you can see what it detects
        print(f"[ESCAPING] Step {escape_counter} | Front: {front_val:.1f}, FL: {fl_val:.1f}, FR: {fr_val:.1f}, Left: {left_val:.1f}, Right: {right_val:.1f}")
        
        # Phase 1 of escape: Reverse for 60 steps (~1 second)
        if escape_counter > 10:
            left_speed = -MAX_SPEED
            right_speed = -MAX_SPEED
        # Phase 2 of escape: Spin right for 30 steps (~0.5 seconds)
        else:
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
            
        escape_counter -= 1
        if escape_counter <= 0:
            print("Escape complete. Resuming normal navigation.")
            is_escaping = False
            
        # Apply speeds and immediately loop (skip normal logic)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        continue

    # ---------------------------------------------
    # 1. NORMAL OBSTACLE AVOIDANCE (Highest Priority)
    # ---------------------------------------------
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    if obstacle_front:
        # If left side is more clear than right side
        if (left_val + fl_val) > (right_val + fr_val):
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
        else:
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
            
    elif obstacle_left:
        left_speed = MAX_SPEED
        right_speed = 0.0
        
    elif obstacle_right:
        left_speed = 0.0
        right_speed = MAX_SPEED

    # ---------------------------------------------
    # 2. TARGET HEADING (Run only if path is clear)
    # ---------------------------------------------
    else:
        if angle_diff > 0.1:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED * 0.5
        elif angle_diff < -0.1:
            left_speed = MAX_SPEED * 0.5
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

    # Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
