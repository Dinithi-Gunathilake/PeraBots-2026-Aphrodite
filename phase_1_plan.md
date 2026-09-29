# Implementation Plan: PeraBots 2026 Phase 1

This document outlines a complete implementation plan for Phase 1 of the PeraBots 2026 University Category, based on the official rulebook. Phase 1 is an online simulation round using Webots (Version R2025a).

## Phase 1 Overview
The challenge is divided into two continuous sections to be completed autonomously:
1.  **Section 1: Obstacle Navigation** - Navigate through a field of 10cm cubic obstacles from a starting zone to a transition zone.
2.  **Section 2: Maze Exploration** - Explore a 4x9 cell maze, locate blue and green tiles at dead ends, determine the majority color, navigate to the end cell, and illuminate the corresponding LED.

---

## Step 1: Environment Setup & Familiarization
1.  **Install Webots**: Ensure you have [Webots R2025a](https://cyberbotics.com/) installed as specified in the rulebook.
2.  **Download Competition Package**: Extract the official Phase 1 package.
3.  **Understand Directory Structure**: 
    *   Do NOT rename or restructure the following core files: 
        *   `TwoWheelRobot.proto`
        *   `controllers/main_controller/`
        *   `main_controller.py`
    *   The `Arena` and `Obstacles` nodes are locked and must not be altered.

## Step 2: Robot Hardware Configuration (`TwoWheelRobot.proto`)
The base robot is provided, but you must equip it with the right sensors to perceive the environment.
*   **Constraints**: Max dimensions are $20 \times 20 \times 20$ cm. Chassis, motors, wheel sizes, physics, and collision geometries cannot be changed. (Minor positional adjustments to the drive wheels are allowed for balance). GPS, Emitters, Receivers, and Supervisor nodes are prohibited.
*   **Pre-installed Hardware**: Ensure `led_blue` and `led_green` remain exactly as they are.
*   **Add Permitted Sensors**:
    1.  **DistanceSensors (Infrared/Sonar)**: Add an array of DistanceSensors around the robot (e.g., Front, Front-Left, Front-Right, Left, Right) to detect walls and obstacles.
    2.  **Camera**: Mount a downward/forward-facing Camera to detect the blue and green tiles on the floor.
    3.  **InertialUnit / Gyroscope / PositionSensors**: Add these to track the robot's orientation and estimate odometry since GPS is forbidden.

## Step 3: Controller Architecture (`main_controller.py`)
Develop a state machine in your `main_controller.py` to handle the different phases of the run autonomously.

### State 1: Obstacle Navigation (Section 1)
*   **Goal**: Navigate from the $25 \times 25$ cm starting zone to the transition zone efficiently and without collisions. Minimum clear passage is 25cm.
*   **Algorithm**: Implement a robust obstacle avoidance algorithm.
    *   **Bug Algorithms (Bug1/Bug2)** or **Vector Field Histogram (VFH)** are good candidates since you don't know the exact map beforehand.
    *   Use DistanceSensors to maintain a safe distance from the 10cm cubic obstacles.
    *   Implement odometry to track your relative progress towards the general direction of the transition zone.

### State 2: Maze Transition & Exploration (Section 2)
*   **Goal**: Navigate the 36-cell maze (cell pitch $25 \times 25$ cm), find colored tiles, count them, and find the end cell.
*   **Algorithm**: 
    *   Implement a maze-solving algorithm, such as **Depth-First Search (DFS)**, **Tremaux's algorithm**, or **Flood Fill** mapping.
    *   Since the maze layout is unknown and efficiency matters, build an internal grid map ($9 \times 4$) using odometry and wall-detection data.

### State 3: Tile Detection & Counting
*   **Goal**: Detect blue and green tiles at dead ends.
*   **Algorithm**: 
    *   Use the Camera feed.
    *   Implement simple color thresholding (e.g., using OpenCV if you install it in your environment, or basic pixel array processing in standard Python) to detect significant blobs of Blue or Green pixels.
    *   Maintain two variables: `blue_count` and `green_count`. Ensure you implement logic to prevent double-counting the same tile if you revisit a dead end.

### State 4: End Cell Navigation & Indication
*   **Goal**: Detect the end cell, completely enter its clear internal area ($23.8 \times 23.8$ cm), stop, and light the correct LED within 3 seconds.
*   **Algorithm**:
    *   Determine the end cell visually or by its structural uniqueness (as per specific arena markers if any).
    *   Stop the motors completely.
    *   Compare `blue_count` and `green_count` to find the majority.
    *   Activate the corresponding LED (`led_blue` or `led_green`) using the Webots API.

---

## Step 4: Software Implementation Rules to Follow
*   **No Hardcoding**: Do not hardcode routes, obstacle positions, or the number/location of tiles. The evaluation arena will have a different layout.
*   **Autonomy**: The run must be 100% autonomous. Do not include keyboard or external input listeners.
*   **Helper Modules**: Any extra Python files or ML models you write must reside inside `controllers/main_controller/`.

## Step 5: Testing & Optimization
*   **Odometry Calibration**: Fine-tune your wheel encoders/InertialUnit math to ensure the robot knows its position accurately over time. Drift will be your biggest enemy.
*   **Edge Cases**: Test what happens if the robot gets stuck in a corner. Implement a "recovery" state to reverse and turn if it detects no movement for several seconds (to avoid the 15-second termination penalty).
*   **Time & Efficiency**: Tweak maximum speeds and turn radii. You are evaluated on completion time and shortest distance traveled.

## Next Steps
To begin programming, please provide the competition package (if you need help setting it up) or let me know if you want to start writing the skeleton for `main_controller.py`!
