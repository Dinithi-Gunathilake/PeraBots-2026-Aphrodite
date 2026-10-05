# Section 2 Implementation Plan: Autonomous Maze Exploration

This document outlines the exact logic we will inject into `main_controller.py` to seamlessly complete the transition into Section 2, fully explore the 36-cell maze, count the colored tiles, and reach the final designated end cell.

## 1. The State Machine
To handle the radically different objectives of Section 1 (Obstacle Avoidance) and Section 2 (Maze Exploration), the controller will be restructured into a State Machine.

*   **`STATE_SECTION_1`**: The robot runs the current A* + Repulsive Steering algorithm. It constantly checks its physical distance to the known Transition Zone coordinate: `(0.22, 1.27)`.
*   **`STATE_TRANSITION`**: Triggered when the robot is within 10cm of the transition cell. It will briefly clear the A* map (to forget the Section 1 obstacles) and activate the downward-facing Camera.
*   **`STATE_EXPLORE`**: The primary maze-solving mode using Frontier Exploration.
*   **`STATE_FINISH`**: Triggered when the maze is 100% explored. The robot routes itself to the final End Cell, comes to a dead stop, and illuminates the LED.

## 2. Hardware Activation (The Camera)
Since the `Camera` node is already physically mounted on the `TwoWheelRobot.proto`, we only need to activate it in the code.
*   The camera will be enabled at the start of `STATE_EXPLORE`.
*   During every simulation step, we will extract the RGB values of the center pixels of the camera feed to determine the floor color beneath the robot.

## 3. Frontier-Based Maze Exploration
Because the maze walls and dead ends are randomized, we cannot use a fixed A* goal. Instead, we use Frontier Exploration to guarantee 100% coverage.
*   **Frontiers**: A "Frontier" is any empty cell on the map that borders an unknown (unmapped) area.
*   **Dynamic Goal**: The A* algorithm will continuously route the robot to the *nearest* Frontier cell.
*   As the robot drives to a Frontier, its distance sensors map the walls around it, revealing new corridors and turning unknown space into known space.
*   Once all corridors and dead ends have been seen, the list of Frontiers becomes empty. This mathematically proves the entire maze has been explored.

## 4. Tile Detection & Counting
While in `STATE_EXPLORE`, the robot actively monitors the camera feed.
*   **Color Thresholding**: If the camera sees strong Blue (`B > R` and `B > G`) or strong Green (`G > R` and `G > B`), it identifies a tile.
*   **Spatial Memory**: To prevent double-counting a tile when the robot spins around or backtracks out of a dead end, it will record the `(X, Y)` coordinate of the detected tile. It will only increment the `blue_count` or `green_count` if the new detection is at least 15cm away from any previously memorized tile.

## 5. End Cell Navigation & LED Indication
*   The End Cell is marked by a Red floor tile.
*   The robot will see *two* Red tiles during its run: The entrance (transition cell) and the exit (end cell).
*   During exploration, if it detects a Red tile that is far away from its starting Transition coordinate `(0.22, 1.27)`, it memorizes it as the `final_end_cell_coordinate`.
*   When the Frontier list empties, it switches to `STATE_FINISH`. It sets its A* goal to the `final_end_cell_coordinate`.
*   Upon arrival, it commands `drive_wheels(0, 0)`, compares the counters, and calls `led_blue.set(1)` or `led_green.set(1)` based on the majority.
