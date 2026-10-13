"""
controller_section2.py

Section 2: Autonomous Maze Exploration & Decision-Making
Explores a 9x4 grid (36 cells), detects blue/green tiles at dead ends,
finds the majority color, navigates to the end cell, and activates the LED.
"""

import math
import collections

TIME_STEP = 16
MAX_WHEEL = 10.0
WHEEL_R = 0.02

# Maze Grid Parameters (from Section 2 specification)
MAZE_ORIGIN_X = 0.220   # Center X (meters) of Cell (0, 0)
MAZE_ORIGIN_Y = 1.455   # Center Y (meters) of Cell (0, 0)
MAZE_CELL_SIZE = 0.250  # Cell pitch (meters)
MAZE_COLS = 9           # Columns (0 to 8)
MAZE_ROWS = 4           # Rows (0 to 3)

def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class Section2Controller:
    def __init__(self, robot):
        self.robot = robot
        
        # Motors
        self.lm = robot.getDevice("left_wheel_motor")
        self.rm = robot.getDevice("right_wheel_motor")
        self.lm.setPosition(float("inf"))
        self.rm.setPosition(float("inf"))
        
        # Encoders & IMU
        self.ls = robot.getDevice("left_wheel_sensor")
        self.rs = robot.getDevice("right_wheel_sensor")
        self.ls.enable(TIME_STEP)
        self.rs.enable(TIME_STEP)
        self.imu = robot.getDevice("inertial_unit")
        self.imu.enable(TIME_STEP)
        
        # Distance Sensors
        sensor_names = [
            "ds_front", "ds_front_left", "ds_left", "ds_back_left",
            "ds_back", "ds_back_right", "ds_right", "ds_front_right"
        ]
        self.ds = [robot.getDevice(name) for name in sensor_names]
        for s in self.ds:
            s.enable(TIME_STEP)
        self.d = [0.8] * 8
        
        # Camera & LEDs
        self.camera = robot.getDevice("camera")
        self.camera.enable(TIME_STEP)
        self.led_blue = robot.getDevice("led_blue")
        self.led_green = robot.getDevice("led_green")
        
        # Odometry & State
        self.x = MAZE_ORIGIN_X
        self.y = MAZE_ORIGIN_Y
        self.th = math.pi / 2
        self.wl_prev = None
        self.wr_prev = None
        
        # Exploration State
        self.state = "INIT_ALIGN"
        self.cx = 0
        self.cy = 0
        self.target_cx = 0
        self.target_cy = 0
        
        self.maze_walls = {}
        self.dfs_visited = {(0, 0)}
        self.dfs_stack = []
        
        # Tile Tracking
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = set()
        self.final_end_cell = None
        self.discrete_path = []
        self.is_finishing = False
        self.completed = False

    def drive_wheels(self, wl, wr):
        self.lm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wl)))
        self.rm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wr)))

    def update_sensors_and_odometry(self):
        for k, dev in enumerate(self.ds):
            v = dev.getValue()
            self.d[k] = 0.8 if v != v else max(0.0, min(0.8, v))
            
        self.th = self.imu.getRollPitchYaw()[2]
        wl, wr = self.ls.getValue(), self.rs.getValue()
        if self.wl_prev is not None and self.wr_prev is not None:
            ds = 0.5 * ((wl - self.wl_prev) + (wr - self.wr_prev)) * WHEEL_R
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        self.wl_prev, self.wr_prev = wl, wr

    def check_floor_tile(self):
        """Processes downward camera feed to detect colored floor tiles."""
        img = self.camera.getImageArray()
        if not img:
            return
            
        w, h = self.camera.getWidth(), self.camera.getHeight()
        r_sum = g_sum = b_sum = 0
        count = 0
        for px_i in range(max(0, w // 2 - 3), min(w, w // 2 + 4)):
            for px_j in range(max(0, h // 2 - 3), min(h, h // 2 + 4)):
                pr, pg, pb = img[px_i][px_j]
                r_sum += pr
                g_sum += pg
                b_sum += pb
                count += 1
                
        r = r_sum / count
        g = g_sum / count
        b = b_sum / count
        
        detected = None
        if b > 28 and b > r * 1.25 and b > g * 1.25:
            detected = "BLUE"
        elif g > 28 and g > r * 1.25 and g > b * 1.25:
            detected = "GREEN"
        elif r > 28 and r > g * 1.25 and r > b * 1.25:
            detected = "RED"
            
        if detected and (self.cx, self.cy) not in self.tile_memory:
            self.tile_memory.add((self.cx, self.cy))
            if detected == "BLUE":
                self.blue_count += 1
                print(f"[CAMERA] Found BLUE tile at ({self.cx}, {self.cy})! Total Blue: {self.blue_count}")
            elif detected == "GREEN":
                self.green_count += 1
                print(f"[CAMERA] Found GREEN tile at ({self.cx}, {self.cy})! Total Green: {self.green_count}")
            elif detected == "RED" and (self.cx > 0 or self.cy > 0):
                self.final_end_cell = (self.cx, self.cy)
                print(f"[CAMERA] Found FINAL RED TILE at ({self.cx}, {self.cy})!")

    def step(self):
        """Executes one step of Section 2 maze solving."""
        self.update_sensors_and_odometry()
        
        if self.state == "INIT_ALIGN":
            # Align North into cell (0, 0)
            err = wrap_angle(math.pi / 2 - self.th)
            if abs(err) > 0.05:
                turn = max(-2.5, min(2.5, 3.5 * err))
                self.drive_wheels(-turn, turn)
            else:
                self.drive_wheels(0, 0)
                self.state = "DFS_SCAN"
                
        elif self.state == "DFS_SCAN":
            self.check_floor_tile()
            
            # 1. Align to exact cardinal heading
            th_w = wrap_angle(self.th)
            if abs(th_w - 0.0) < 0.78:
                cardinal_th = 0.0
                f_dir, l_dir, r_dir, b_dir = "E", "N", "S", "W"
            elif abs(wrap_angle(th_w - math.pi / 2)) < 0.78:
                cardinal_th = math.pi / 2
                f_dir, l_dir, r_dir, b_dir = "N", "W", "E", "S"
            elif abs(wrap_angle(th_w - math.pi)) < 0.78:
                cardinal_th = math.pi
                f_dir, l_dir, r_dir, b_dir = "W", "S", "N", "E"
            else:
                cardinal_th = -math.pi / 2
                f_dir, l_dir, r_dir, b_dir = "S", "E", "W", "N"

            align_err = wrap_angle(cardinal_th - self.th)
            if abs(align_err) > 0.04:
                turn = max(-2.0, min(2.0, 3.5 * align_err))
                if abs(turn) < 0.8: turn = 0.8 if turn > 0 else -0.8
                self.drive_wheels(-turn, turn)
                return True
            self.drive_wheels(0, 0)

            # 2. Map walls
            if (self.cx, self.cy) not in self.maze_walls:
                WALL_THRESH_FRONT = 0.180
                WALL_THRESH_SIDE  = 0.125
                walls = {
                    f_dir: (self.d[0] < WALL_THRESH_FRONT),
                    l_dir: (self.d[2] < WALL_THRESH_SIDE),
                    r_dir: (self.d[6] < WALL_THRESH_SIDE),
                    b_dir: False
                }
                if self.cy == 3: walls["N"] = True
                if self.cy == 0: walls["S"] = True
                if self.cx == 0: walls["W"] = True
                if self.cx == 8: walls["E"] = True
                if self.cx == 0 and self.cy == 0: walls["S"] = True
                
                self.maze_walls[(self.cx, self.cy)] = walls
                print(f"[WALLS] ({self.cx}, {self.cy}) mapped: {walls}")
                
                # Consistency
                dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
                opp_dir = {"N": "S", "S": "N", "E": "W", "W": "E"}
                for d_name, has_wall in walls.items():
                    dx, dy = dir_deltas[d_name]
                    nx, ny = self.cx + dx, self.cy + dy
                    if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS and (nx, ny) in self.maze_walls:
                        if not has_wall:
                            self.maze_walls[(nx, ny)][opp_dir[d_name]] = False

            # 3. Choose DFS move
            dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
            walls = self.maze_walls[(self.cx, self.cy)]
            unvisited = []
            for d_name in ["N", "E", "S", "W"]:
                if not walls.get(d_name, True):
                    dx, dy = dir_deltas[d_name]
                    nx, ny = self.cx + dx, self.cy + dy
                    if 0 <= nx < MAZE_COLS and 0 <= ny < MAZE_ROWS:
                        if (nx, ny) not in self.dfs_visited:
                            unvisited.append(((nx, ny), d_name))
                            
            if unvisited:
                next_cell, d_name = unvisited[0]
                self.dfs_visited.add(next_cell)
                self.dfs_stack.append((self.cx, self.cy))
                self.target_cx, self.target_cy = next_cell
                self.state = "DFS_TURN"
            else:
                if not self.dfs_stack:
                    print("=== DFS COMPLETE! All reachable cells explored ===")
                    self.state = "FINISH"
                else:
                    back_cell = self.dfs_stack.pop()
                    self.target_cx, self.target_cy = back_cell
                    self.state = "DFS_TURN"
                    
        elif self.state == "DFS_TURN":
            if abs(self.target_cx - self.cx) > 0:
                target_angle = 0.0 if self.target_cx > self.cx else math.pi
            else:
                target_angle = math.pi / 2 if self.target_cy > self.cy else -math.pi / 2
                
            e = wrap_angle(target_angle - self.th)
            if abs(e) > 0.08:
                turn = max(-2.5, min(2.5, 3.5 * e))
                if abs(turn) < 1.0: turn = 1.0 if turn > 0 else -1.0
                self.drive_wheels(-turn, turn)
            else:
                self.drive_wheels(0, 0)
                self.state = "DFS_FORWARD"
                
        elif self.state == "DFS_FORWARD":
            target_x = MAZE_ORIGIN_X + self.target_cx * MAZE_CELL_SIZE
            target_y = MAZE_ORIGIN_Y + self.target_cy * MAZE_CELL_SIZE
            
            if self.target_cx > self.cx: rem = target_x - self.x
            elif self.target_cx < self.cx: rem = self.x - target_x
            elif self.target_cy > self.cy: rem = target_y - self.y
            else: rem = self.y - target_y

            has_wall_ahead = (self.d[0] < 0.18)
            arrived = (rem <= 0.015) or (has_wall_ahead and rem < 0.030 and self.d[0] <= 0.050)
            
            if arrived:
                self.drive_wheels(0, 0)
                self.cx = self.target_cx
                self.cy = self.target_cy
                self.x = target_x
                self.y = target_y
                self.state = "FINISH_NAV" if self.is_finishing else "DFS_SCAN"
            else:
                speed = 4.0
                if abs(self.target_cx - self.cx) > 0:
                    nominal_y = MAZE_ORIGIN_Y + self.target_cy * MAZE_CELL_SIZE
                    base_angle = 0.0 if self.target_cx > self.cx else math.pi
                    lat_err = nominal_y - self.y
                    correction = max(-0.35, min(0.35, lat_err * 4.0))
                    target_angle = wrap_angle(base_angle + (correction if base_angle == 0.0 else -correction))
                else:
                    nominal_x = MAZE_ORIGIN_X + self.target_cx * MAZE_CELL_SIZE
                    base_angle = math.pi / 2 if self.target_cy > self.cy else -math.pi / 2
                    lat_err = nominal_x - self.x
                    correction = max(-0.35, min(0.35, lat_err * 4.0))
                    target_angle = wrap_angle(base_angle - (correction if base_angle > 0 else -correction))
                    
                heading_err = wrap_angle(target_angle - self.th)
                turn = heading_err * 4.0
                if self.d[2] < 0.14 and self.d[6] < 0.14:
                    turn += (self.d[2] - self.d[6]) * 3.0
                turn = max(-2.0, min(2.0, turn))
                self.drive_wheels(speed - turn, speed + turn)
                
        elif self.state == "FINISH":
            if not self.final_end_cell:
                self.final_end_cell = (8, 3)
            
            # BFS path to end cell
            queue = collections.deque([((self.cx, self.cy), [])])
            visited = {(self.cx, self.cy)}
            self.discrete_path = []
            dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
            
            while queue:
                curr, path = queue.popleft()
                if curr == self.final_end_cell:
                    self.discrete_path = path
                    break
                walls = self.maze_walls.get(curr, {})
                for d_name, (dx, dy) in dir_deltas.items():
                    if not walls.get(d_name, True):
                        nxt = (curr[0] + dx, curr[1] + dy)
                        if 0 <= nxt[0] < MAZE_COLS and 0 <= nxt[1] < MAZE_ROWS and nxt not in visited:
                            visited.add(nxt)
                            queue.append((nxt, path + [nxt]))
                            
            self.is_finishing = True
            self.state = "FINISH_NAV"
            
        elif self.state == "FINISH_NAV":
            if not self.discrete_path:
                self.drive_wheels(0, 0)
                print("========================================")
                print("===      CHALLENGE COMPLETED!        ===")
                print(f"=== Total Blue: {self.blue_count}, Total Green: {self.green_count} ===")
                if self.blue_count > self.green_count:
                    print("=== Majority: BLUE -> Turning on Blue LED ===")
                    self.led_blue.set(1)
                else:
                    print("=== Majority: GREEN -> Turning on Green LED ===")
                    self.led_green.set(1)
                print("========================================")
                self.completed = True
                return False
                
            self.target_cx, self.target_cy = self.discrete_path.pop(0)
            self.state = "DFS_TURN"
            
        return True

    def is_completed(self):
        return self.completed
