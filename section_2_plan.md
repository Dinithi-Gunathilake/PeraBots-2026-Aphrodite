# Section 2 Implementation Plan: Frontier Flood-Fill Autonomous Exploration

This document outlines the architecture to replace Section 2 in `main_controller.py` with a **Frontier-Based Flood-Fill / BFS Exploration System** tailored to the competition requirements and the `TwoWheelRobot.proto` hardware.

---

## 1. System Invariants & Grid Representation

* **Maze Grid**: 4 rows $\times$ 9 columns ($36$ cells total).
  * Column indices: `col` $\in [0, 8]$ ($X = 0.22\text{ m} + \text{col} \times 0.25\text{ m}$).
  * Row indices: `row` $\in [0, 3]$ ($Y = 1.455\text{ m} + \text{row} \times 0.25\text{ m}$).
  * Clear interior width per cell: $\approx 23.8\text{ cm}$.
* **Edge States**: Each passage between adjacent cells has one of three states:
  * `"unknown"`: Not yet observed by distance sensors.
  * `"wall"`: Confirmed closed wall.
  * `"open"`: Confirmed traversable passage.
  * **Bidirectional consistency**: Any update to edge $(c, r) \to \text{dir}$ immediately updates the opposite edge of the neighbor.
* **Exploration Completion Invariant**:
  Exploration is complete **only** when all reachable cells have no `"unknown"` edges.

---

## 2. Early Red End Tile Discovery & Return Policy

> [!IMPORTANT]
> **Never Stop Early on the Red Tile**  
> If the robot enters the designated Red end tile cell while unvisited cells or unresolved passages still exist, it must **memorize the position, continue exploration, and return later**.

* **Memorization**:
  When the camera detects the Red tile at $(col, row) \neq (0, 0)$:
  - Record `red_cell_coord = (col, row)` in persistent memory.
  - Mark `cell.color = "red"` and `cell.visited = True`.
  - Log: `[CAMERA] Memorized FINAL RED TILE at (col, row)! Continuing exploration...`
* **Continuation**:
  - The robot does **not** stop or switch to finish mode.
  - It treats the Red cell as a normal visited node and queries the BFS planner for the nearest unresolved frontier.
  - It systematically completes the exploration of all remaining unvisited corridors and cells.
* **Return Navigation**:
  - When all reachable frontiers and passages are 100% resolved:
  - Run BFS from the robot's current cell over known open corridors directly to `red_cell_coord`.
  - Follow the computed shortest path back to the Red end cell.
  - Stop at the exact center of the Red cell, illuminate the majority color LED (`led_blue` or `led_green`), and turn off motors.

---

## 3. Modular Architecture

1. **`MazeMap` Class**:
   - Represents the $4 \times 9$ grid of `Cell` objects.
   - Enforces outer boundary walls and seals the transition door back to Section 1.
   - Updates edges bidirectionally.
   - Caches observed tile colors (`"blue"`, `"green"`, `"red"`) and stores `red_cell_coord`.
2. **`FloodFillPlanner` (BFS Engine)**:
   - When immediate unvisited passages exist: prioritizes immediate forward movement.
   - When in a dead end or fully visited junction: runs BFS over known `"open"` passages to find the shortest path to the **nearest cell with an unresolved edge / unvisited neighbor**.
   - Generates a step-by-step cardinal path $[(c_1, r_1), (c_2, r_2), \dots]$.
   - Generates the final BFS route to `red_cell_coord` upon exploration completion.
3. **`MotionController`**:
   - `turn_to_cardinal(heading)`: Closed-loop turn using IMU yaw feedback.
   - `move_one_cell()`: Closed-loop drive along cardinal line with IMU heading lock and dual parallel-wall centering.
   - Odometry snap to exact cell center upon arrival.
4. **`State Machine` Flow**:
   - `S2_INSPECT`: Inspect floor color and 4 cardinal distance sensors; update map. (If Red: memorize coordinate and keep exploring).
   - `S2_PLAN`: Query planner for next cell on route to nearest frontier (or red cell if finished).
   - `S2_TURN`: Align robot with target direction.
   - `S2_MOVE`: Drive forward one cell to target.
   - `S2_FINISH_ROUTE`: BFS navigation back to Red cell after 100% exploration.
   - `S2_FINISH_ALIGN`: Stop at center of Red cell, determine majority color, illuminate LED (`led_blue` or `led_green`), stop motors.

---

## 4. Migration Roadmap

1. **Phase 1**: Add `Cell`, `MazeMap`, and BFS pathfinding functions as clean, isolated helpers.
2. **Phase 2**: Replace Section 2 state machine (lines 473–735 of `main_controller.py`) with the new modular states (`S2_INSPECT`, `S2_PLAN`, `S2_TURN`, `S2_MOVE`, `S2_FINISH_ROUTE`, `S2_FINISH_ALIGN`).
3. **Phase 3**: Connect the camera color detector and 8-sensor distance mapping to `S2_INSPECT`.
4. **Phase 4**: Test in Webots to verify 100% cell coverage, accurate single-count of Blue/Green tiles, early Red tile memorization, return to Red cell, and LED activation.
