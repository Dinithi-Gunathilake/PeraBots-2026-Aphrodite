"""
main_controller.py

PeraBots 2026 - Phase 1 - Complete Advanced SLAM
Step 4: A* Global Planning + 8-Sensor Occupancy Grid
"""

from controller import Robot
import math
import heapq
import collections

TIME_STEP = 16
MAX_WHEEL = 10.0
WHEEL_R = 0.02
ARENA_W, ARENA_H = 2.44, 1.22
GOAL = (0.17, 1.29)

CELL = 0.02
# Section 1 is 2.44 wide, Section 2 is 2.25 wide (total ~4.7m).
MIN_X, MAX_X = -3.0, 3.0
MIN_Y, MAX_Y = -1.0, 3.0
NX = int(round((MAX_X - MIN_X) / CELL))
NY = int(round((MAX_Y - MIN_Y) / CELL))
OFFSET_X = int(round(-MIN_X / CELL))
OFFSET_Y = int(round(-MIN_Y / CELL))
INF = 1e18

# 8 Sensors (Name, X_offset, Y_offset, Angle_in_degrees)
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

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def fit_wall(pts, centre, half):
    sel = [(p, 1.0 / r) for (p, r) in pts if abs(wrap(p - centre)) <= half]
    A = B = 0.0
    for _ in range(3):
        if len(sel) < 4: return None
        sxx = sxy = syy = sxu = syu = 0.0
        for p, u in sel:
            c, s = math.cos(p), math.sin(p)
            sxx += c * c; sxy += c * s; syy += s * s; sxu += c * u; syu += s * u
        det = sxx * syy - sxy * sxy
        if det < 1e-9: return None
        A = (sxu * syy - syu * sxy) / det
        B = (syu * sxx - sxu * sxy) / det
        keep = [(p, u) for (p, u) in sel if abs(A * math.cos(p) + B * math.sin(p) - u) < 0.08 * u]
        if len(keep) == len(sel): break
        sel = keep
    if math.hypot(A, B) < 1e-6: return None
    return math.atan2(B, A), 1.0 / math.hypot(A, B)

def localise_from_scan(samples):
    pts = [(p, d + 0.042) for (p, d) in samples if d < 0.75]
    if len(pts) < 20: return None
    pm, _ = min(pts, key=lambda q: q[1])
    w1 = fit_wall(pts, pm, 0.35)
    if w1 is None: return None
    a1, D1 = w1
    best = None
    for sgn in (-1, 1):
        c = wrap(a1 + sgn * math.pi / 2)
        w = fit_wall(pts, c, 0.21)
        if w is not None and abs(wrap(w[0] - c)) < 0.15:
            if best is None or w[1] < best[1][1]: best = (sgn, w)
    if best is None: return None
    sgn, (a2, D2) = best
    if sgn == -1: phi_x, Dx, Dy = a1, D1, D2
    else: phi_x, Dx, Dy = a2, D2, D1
    return 2.44 - 0.006 - Dx, 0.006 + Dy, phi_x

INFLATION_OFFSETS = tuple((dx, dy) for dx in range(-4, 5) for dy in range(-4, 5) if dx*dx + dy*dy <= 16)

class Bot:
    def __init__(self, robot):
        self.robot = robot
        self.lm = robot.getDevice('left_wheel_motor')
        self.rm = robot.getDevice('right_wheel_motor')
        self.lm.setPosition(float('inf'))
        self.rm.setPosition(float('inf'))
        
        self.ls = robot.getDevice('left_wheel_sensor')
        self.rs = robot.getDevice('right_wheel_sensor')
        self.ls.enable(TIME_STEP)
        self.rs.enable(TIME_STEP)
        
        self.imu = robot.getDevice('inertial_unit')
        self.imu.enable(TIME_STEP)
        
        self.ds = []
        for name, x, y, a in SENSORS:
            dev = robot.getDevice(name)
            dev.enable(TIME_STEP)
            self.ds.append((dev, x, y, math.radians(a)))
            
        self.d = [0.8] * 8
        self.x = self.y = self.th = 0.0
        self.wl_prev = self.wr_prev = None
        
        # SLAM Maps
        self.map = bytearray([127] * (NX * NY)) # 127=Unknown, 0=Free, 255=Obstacle
        self.inflated_map = bytearray(NX * NY)
        self.path = []
        self.stuck_counter = 0
        self.prev_x = 0.0
        self.prev_y = 0.0
        
        # State Machine & Hardware
        self.state = "SECTION_1"
        self.camera = robot.getDevice("camera")
        self.led_blue = robot.getDevice("led_blue")
        self.led_green = robot.getDevice("led_green")
        
        # Color Counting & Memory
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = []
        self.final_end_cell = None
        self.step_count = 0

    def drive_wheels(self, wl, wr):
        self.lm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wl)))
        self.rm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wr)))

    def step(self):
        if self.robot.step(TIME_STEP) == -1: return False
        
        # Read Sensors
        for k, (dev, _, _, _) in enumerate(self.ds):
            v = dev.getValue()
            self.d[k] = 0.8 if v != v else max(0.0, min(0.8, v))
            
        # Update Odometry
        self.th = self.imu.getRollPitchYaw()[2]
        wl, wr = self.ls.getValue(), self.rs.getValue()
        if self.wl_prev is not None:
            ds = 0.5 * ((wl - self.wl_prev) + (wr - self.wr_prev)) * WHEEL_R
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        self.wl_prev, self.wr_prev = wl, wr
        
        self.wl_prev, self.wr_prev = wl, wr
        
        if self.state == "SECTION_1":
            self.scan_match()
            self.update_map()
        
        self.step_count += 1
        return True

    def scan_match(self):
        for k, (_, _, _, ang) in enumerate(self.ds):
            d = self.d[k]
            if d >= 0.7: continue
            
            abs_angle = wrap(self.th + ang)
            
            # Smart Scan Matching using cosine projection for slight angles
            # If pointing roughly Right (+X)
            if abs(wrap(abs_angle - 0.0)) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - 0.0))
                pred_x = ARENA_W - 0.006 - perp_d - 0.042
                if abs(pred_x - self.x) < 0.05: 
                    self.x = 0.90 * self.x + 0.10 * pred_x
            
            # Roughly Top (+Y)
            elif abs(wrap(abs_angle - (math.pi/2))) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - (math.pi/2)))
                pred_y = ARENA_H - 0.006 - perp_d - 0.042
                if abs(pred_y - self.y) < 0.05: 
                    self.y = 0.90 * self.y + 0.10 * pred_y
                    
            # Roughly Left (-X)
            elif abs(wrap(abs_angle - math.pi)) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - math.pi))
                pred_x = 0.006 + perp_d + 0.042
                if abs(pred_x - self.x) < 0.05:
                    self.x = 0.90 * self.x + 0.10 * pred_x
                    
            # Roughly Bottom (-Y)
            elif abs(wrap(abs_angle - (-math.pi/2))) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - (-math.pi/2)))
                pred_y = 0.006 + perp_d + 0.042
                if abs(pred_y - self.y) < 0.05:
                    self.y = 0.90 * self.y + 0.10 * pred_y

    def update_map(self):
        cx = int(self.x / CELL) + OFFSET_X
        cy = int(self.y / CELL) + OFFSET_Y
        
        # 1. Sweep a solid physical footprint of contiguous Free Space (radius = 5 cells = 10cm)
        # This prevents the BFS from being choked by thin raycast lines and inflation!
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                if dx*dx + dy*dy <= 25:
                    ix, iy = cx+dx, cy+dy
                    if 0 <= ix < NX and 0 <= iy < NY:
                        if self.map[iy * NX + ix] == 127:
                            self.map[iy * NX + ix] = 0
                            
        # 2. Sensor Raycasting
        for k, (_, sx, sy, ang) in enumerate(self.ds):
            d = self.d[k]
            abs_a = wrap(self.th + ang)
            
            # Max range for raycast clearing
            clear_d = d if d < 0.75 else 0.75
            
            # Bresenham to clear free space
            px_clear = self.x + clear_d * math.cos(abs_a)
            py_clear = self.y + clear_d * math.sin(abs_a)
            
            x0, y0 = cx, cy
            x1 = int(px_clear / CELL) + OFFSET_X
            y1 = int(py_clear / CELL) + OFFSET_Y
            
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx_step = 1 if x0 < x1 else -1
            sy_step = 1 if y0 < y1 else -1
            err = dx - dy
            
            while True:
                if 0 <= x0 < NX and 0 <= y0 < NY:
                    # Clear free space ONLY if it's currently Unknown (don't erase walls!)
                    if self.map[y0 * NX + x0] == 127:
                        self.map[y0 * NX + x0] = 0 
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx_step
                if e2 < dx:
                    err += dx
                    y0 += sy_step
                    
            # Mark Obstacle
            if d < 0.75:
                px = self.x + (d + 0.042) * math.cos(abs_a)
                py = self.y + (d + 0.042) * math.sin(abs_a)
                ix = int(px / CELL) + OFFSET_X
                iy = int(py / CELL) + OFFSET_Y
                if 0 <= ix < NX and 0 <= iy < NY:
                    self.map[iy * NX + ix] = 255
                    for dx, dy in INFLATION_OFFSETS:
                        ox, oy = ix + dx, iy + dy
                        if 0 <= ox < NX and 0 <= oy < NY:
                            self.inflated_map[oy * NX + ox] = 255

    def plan_to_frontier(self):
        """BFS to find the closest reachable frontier cell."""
        cx = int(self.x / CELL) + OFFSET_X
        cy = int(self.y / CELL) + OFFSET_Y
        if not (0 <= cx < NX and 0 <= cy < NY): return
        
        queue = collections.deque([((cx, cy), [])])
        visited = bytearray(NX * NY)
        visited[cy * NX + cx] = 1
        
        while queue:
            (px, py), path = queue.popleft()
            
            # Is it a frontier? (Free space bordering Unknown)
            is_frontier = False
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = px+dx, py+dy
                if 0 <= nx < NX and 0 <= ny < NY:
                    if self.map[ny * NX + nx] == 127:
                        is_frontier = True
                        break
                        
            if is_frontier and len(path) > 2:
                # Target acquired! Convert grid path to real-world coordinates
                self.path = [((ix - OFFSET_X + 0.5)*CELL, (iy - OFFSET_Y + 0.5)*CELL) for ix, iy in path]
                return
                
            # Expand neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = px+dx, py+dy
                if 0 <= nx < NX and 0 <= ny < NY:
                    nidx = ny * NX + nx
                    if not visited[nidx]:
                        visited[nidx] = 1
                        if self.map[nidx] == 0: # Only route through known Free space
                            if self.inflated_map[nidx] != 255:
                                queue.append(((nx, ny), path + [(nx, ny)]))
                                
        print("[EXPLORE] MAZE FULLY MAPPED! No frontiers left.")
        self.path = []
                
    def a_star(self):
        """Find fastest path to GOAL avoiding self.map obstacles."""
        start = (int(self.x / CELL) + OFFSET_X, int(self.y / CELL) + OFFSET_Y)
        goal = (int(GOAL[0] / CELL) + OFFSET_X, int(GOAL[1] / CELL) + OFFSET_Y)
        
        # Cap to array bounds
        if not (0 <= start[0] < NX and 0 <= start[1] < NY): return
        
        N = NX * NY
        moves = [(1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0), 
                 (1,1,1.414), (1,-1,1.414), (-1,1,1.414), (-1,-1,1.414)]
                 
        for check_grid in (self.inflated_map, self.map):
            g = [INF] * N
            parent = [-1] * N
            s_idx = start[1] * NX + start[0]
            g[s_idx] = 0.0
            heap = [(0.0, s_idx)]
            closed = bytearray(N)
            
            while heap:
                _, idx = heapq.heappop(heap)
                if closed[idx]: continue
                closed[idx] = 1
                i, j = idx % NX, idx // NX
                
                if i == goal[0] and j == goal[1]:
                    out = []
                    while idx != -1:
                        out.append((idx % NX, idx // NX))
                        idx = parent[idx]
                    out.reverse()
                    self.path = [((ix - OFFSET_X + 0.5)*CELL, (iy - OFFSET_Y + 0.5)*CELL) for ix, iy in out]
                    return
                    
                for di, dj, cost in moves:
                    ni, nj = i + di, j + dj
                    if ni < 0 or nj < 0 or ni >= NX or nj >= NY: continue
                    nidx = nj * NX + ni
                    if check_grid[nidx] == 255 or closed[nidx]: continue
                    
                    ng = g[idx] + cost
                    if ng < g[nidx]:
                        g[nidx] = ng
                        parent[nidx] = idx
                        h = math.hypot(ni - goal[0], nj - goal[1])
                        heapq.heappush(heap, (ng + h, nidx))
                        
        print(f"[A* ERROR] Queue emptied. No path found to GOAL {goal} from START {start}!")
        self.path = []

    def navigate(self):
        """Pure Pursuit Follower to drive along self.path with collision avoidance."""
        if not self.path or len(self.path) < 2:
            print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
            self.drive_wheels(0, 0)
            return

        # Emergency obstacle proximity: back up slightly if directly against an obstacle
        if min(self.d[0], self.d[1], self.d[7]) < 0.045:
            self.drive_wheels(-MAX_WHEEL * 0.4, -MAX_WHEEL * 0.4)
            return
            
        # Find closest point on path to current robot position
        min_dist = INF
        closest_idx = 0
        for i, p in enumerate(self.path):
            d = math.hypot(p[0] - self.x, p[1] - self.y)
            if d < min_dist:
                min_dist = d
                closest_idx = i
                
        # Lookahead 6 nodes (12cm) to prevent corner cutting
        lookahead = min(closest_idx + 6, len(self.path) - 1)
        target = self.path[lookahead]
        
        # Calculate heading error
        target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
        e = wrap(target_angle - self.th)
        
        # Drive control - Smoother!
        if abs(e) > 0.6: # Sharp turn: spin in place
            v_l = MAX_WHEEL * 0.5 if e < 0 else -MAX_WHEEL * 0.5
            v_r = -MAX_WHEEL * 0.5 if e < 0 else MAX_WHEEL * 0.5
        else: # Smooth proportional curve
            front_min = min(self.d[0], self.d[1], self.d[7])
            if front_min < 0.12:
                speed = MAX_WHEEL * 0.45
            elif front_min < 0.20:
                speed = MAX_WHEEL * 0.70
            else:
                speed = MAX_WHEEL * 0.85
                
            turn = e * MAX_WHEEL * 0.80  # steering gain
            
            # Repulsive Steering: Push away from close corners to prevent clipping!
            if self.d[1] < 0.10: # Front-Left sees corner
                turn -= (0.10 - self.d[1]) * MAX_WHEEL * 2.5
            if self.d[7] < 0.10: # Front-Right sees corner
                turn += (0.10 - self.d[7]) * MAX_WHEEL * 2.5
            if self.d[2] < 0.06: # Left flank
                turn -= (0.06 - self.d[2]) * MAX_WHEEL * 2.0
            if self.d[6] < 0.06: # Right flank
                turn += (0.06 - self.d[6]) * MAX_WHEEL * 2.0
                
            v_l = speed - turn
            v_r = speed + turn
        self.drive_wheels(v_l, v_r)

# ==============================================================================
# SECTION 2: MAP REPRESENTATION & BFS EXPLORATION ENGINE
# ==============================================================================

DIR_OFFSET = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
OPP_DIR = {"N": "S", "S": "N", "E": "W", "W": "E"}

def angle_to_dir(a):
    a = wrap(a)
    if abs(a) < 0.785:
        return "E"
    if abs(wrap(a - math.pi / 2)) < 0.785:
        return "N"
    if abs(wrap(a - math.pi)) < 0.785:
        return "W"
    if abs(wrap(a + math.pi / 2)) < 0.785:
        return "S"
    return "E"

class Cell:
    def __init__(self, col, row):
        self.col = col
        self.row = row
        self.visited = False
        self.scanned = False
        self.color = "unknown"  # "none", "BLUE", "GREEN", "RED"
        self.edges = {
            "N": "unknown",
            "S": "unknown",
            "E": "unknown",
            "W": "unknown"
        }

class MazeMap:
    def __init__(self):
        self.grid = {}
        for c in range(9):
            for r in range(4):
                self.grid[(c, r)] = Cell(c, r)
        
        # Outer boundary perimeter walls
        for c in range(9):
            self.grid[(c, 0)].edges["S"] = "wall"
            self.grid[(c, 3)].edges["N"] = "wall"
        for r in range(4):
            self.grid[(0, r)].edges["W"] = "wall"
            self.grid[(8, r)].edges["E"] = "wall"
            
        # Transition doorway South of (0, 0) is closed
        self.grid[(0, 0)].edges["S"] = "wall"
        
        self.red_cell_coord = None
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = set()

    def set_edge(self, c, r, d, state):
        if (c, r) not in self.grid:
            return
        # Outer boundaries can never be opened
        if c == 0 and d == "W": return
        if c == 8 and d == "E": return
        if r == 0 and d == "S": return
        if r == 3 and d == "N": return
        if c == 0 and r == 0 and d == "S": return

        self.grid[(c, r)].edges[d] = state
        
        dx, dy = DIR_OFFSET[d]
        nc, nr = c + dx, r + dy
        if (nc, nr) in self.grid:
            self.grid[(nc, nr)].edges[OPP_DIR[d]] = state

    def record_color(self, c, r, detected_color):
        if (c, r) in self.tile_memory:
            return
        
        cell = self.grid.get((c, r))
        if cell:
            cell.color = detected_color
            
        if detected_color in ("BLUE", "GREEN", "RED"):
            self.tile_memory.add((c, r))
            if detected_color == "BLUE":
                self.blue_count += 1
                print(f"[CAMERA] Found BLUE tile at ({c}, {r})! Total Blue: {self.blue_count}")
            elif detected_color == "GREEN":
                self.green_count += 1
                print(f"[CAMERA] Found GREEN tile at ({c}, {r})! Total Green: {self.green_count}")
            elif detected_color == "RED":
                if c > 0 or r > 0:
                    self.red_cell_coord = (c, r)
                    print(f"[CAMERA] Memorized FINAL RED TILE at discrete ({c}, {r})! Continuing exploration...")

    def find_nearest_unvisited_path(self, start_c, start_r):
        """
        Finds the shortest path over confirmed 'open' edges from (start_c, start_r)
        to the nearest unvisited cell.
        Returns a list of cell coordinates: [(c1, r1), (c2, r2), ...] or None if all reachable cells visited.
        """
        queue = collections.deque([(start_c, start_r, [])])
        visited_nodes = set([(start_c, start_r)])
        
        while queue:
            c, r, path = queue.popleft()
            cell = self.grid.get((c, r))
            if not cell:
                continue
                
            for d in ["N", "S", "W", "E"]:
                if cell.edges.get(d) == "open":
                    dx, dy = DIR_OFFSET[d]
                    nc, nr = c + dx, r + dy
                    if (nc, nr) in self.grid and (nc, nr) not in visited_nodes:
                        visited_nodes.add((nc, nr))
                        nbr = self.grid[(nc, nr)]
                        step_path = path + [(nc, nr)]
                        if not nbr.visited:
                            return step_path
                        else:
                            queue.append((nc, nr, step_path))
        return None

    def find_path_to(self, start_c, start_r, target_c, target_r):
        """
        Finds shortest path over confirmed 'open' edges from start to target.
        Returns list of cell coordinates: [(c1, r1), (c2, r2), ...] or None if unreachable.
        """
        if (start_c, start_r) == (target_c, target_r):
            return []
            
        queue = collections.deque([(start_c, start_r, [])])
        visited_nodes = set([(start_c, start_r)])
        
        while queue:
            c, r, path = queue.popleft()
            if (c, r) == (target_c, target_r):
                return path
                
            cell = self.grid.get((c, r))
            if not cell:
                continue
                
            for d in ["N", "S", "W", "E"]:
                if cell.edges.get(d) == "open":
                    dx, dy = DIR_OFFSET[d]
                    nc, nr = c + dx, r + dy
                    if (nc, nr) in self.grid and (nc, nr) not in visited_nodes:
                        visited_nodes.add((nc, nr))
                        step_path = path + [(nc, nr)]
                        if (nc, nr) == (target_c, target_r):
                            return step_path
                        queue.append((nc, nr, step_path))
        return None

def main():
    global GOAL
    robot = Robot()
    bot = Bot(robot)
    
    # 0. Read initial environment
    if not bot.step(): return
    
    # Standalone Section 2 Check:
    # 1. Inside 25cm channel (chamber or cell 0,0): Left and Right walls close (d[2] < 0.15 and d[6] < 0.15)
    # 2. At doorway threshold facing North: Front-Left and Front-Right doorway walls close (d[1] < 0.15 and d[7] < 0.15)
    is_sec2_corridor = (bot.d[2] < 0.15 and bot.d[6] < 0.15 and (bot.d[2] + bot.d[6]) < 0.25)
    is_sec2_doorway = (bot.d[1] < 0.15 and bot.d[7] < 0.15 and bot.d[0] > 0.15 and abs(wrap(bot.th - math.pi/2)) < 0.5)
    is_sec2_start = is_sec2_corridor or is_sec2_doorway
    
    if is_sec2_start:
        print("=== STANDALONE SECTION 2 EVALUATION DETECTED ===")
        bot.camera.enable(TIME_STEP)
        bot.maze_start_x = 0.22
        bot.maze_start_y = 1.455
        
        # Snap yaw to closest cardinal heading (normally North = pi/2)
        cardinals = [0.0, math.pi / 2, math.pi, -math.pi / 2]
        bot.th = min(cardinals, key=lambda a: abs(wrap(bot.th - a)))
        
        # If back sensor is open (d[4] > 0.30) or at doorway, robot advances into Cell (0, 0)
        # If back sensor sees close wall (d[4] < 0.20), robot is already inside Maze Cell (0, 0)
        if is_sec2_doorway or bot.d[4] > 0.30:
            print("[INIT] Placed at Entrance / Transition Chamber. Advancing into Maze Cell (0, 0)...")
            bot.x = 0.22
            bot.y = 1.275
            bot.state = "TRANSITION"
        else:
            print("[INIT] Placed directly inside Maze Cell (0, 0). Starting exploration!")
            bot.x = 0.22
            bot.y = 1.455
            bot.maze = MazeMap()
            bot.col = 0
            bot.row = 0
            bot.target_col = None
            bot.target_row = None
            bot.active_path = []
            bot.is_finishing = False
            bot.state = "S2_INSPECT"
    else:
        print("--- Starting Auto-Calibration Spin ---")
        bot.drive_wheels(MAX_WHEEL, -MAX_WHEEL)
        
        samples = []
        for _ in range(180):
            if not bot.step(): return
            samples.append((bot.th, bot.d[0]))
            
        bot.drive_wheels(0, 0)
        
        result = localise_from_scan(samples)
        if result:
            bot.x, bot.y, _ = result
            print(f"Calibrated: X={bot.x:.3f}, Y={bot.y:.3f}")
            bot.map = bytearray([127] * (NX * NY))
            bot.inflated_map = bytearray(NX * NY)
            print("--- Navigating Maze to Goal ---")
            bot.a_star() # Force A* immediately on step 0!
        else:
            print("[FALLBACK] Section 1 outer walls not matched. Switching to Standalone Section 2 mode!")
            bot.camera.enable(TIME_STEP)
            bot.maze_start_x = 0.22
            bot.maze_start_y = 1.455
            bot.x = 0.22
            bot.y = 1.275
            bot.th = math.pi / 2
            bot.state = "TRANSITION"
            
    plan_counter = 0
    bot.prev_x, bot.prev_y = bot.x, bot.y
    while bot.step():
        if bot.state == "SECTION_1":
            plan_counter += 1
            if plan_counter % 10 == 0 or bot.d[0] < 0.15:
                bot.a_star()
            bot.navigate()
            
            # Anti-stuck monitoring: check progress every 30 steps
            if plan_counter % 30 == 0:
                moved = math.hypot(bot.x - bot.prev_x, bot.y - bot.prev_y)
                if moved < 0.015:
                    bot.stuck_counter += 1
                else:
                    bot.stuck_counter = 0
                bot.prev_x, bot.prev_y = bot.x, bot.y
                
            if bot.stuck_counter >= 3:
                print("[NAVIGATE] Stuck detected! Reversing and re-planning...")
                for _ in range(15):
                    bot.drive_wheels(-MAX_WHEEL * 0.5, -MAX_WHEEL * 0.5)
                    bot.step()
                for _ in range(15):
                    bot.drive_wheels(-MAX_WHEEL * 0.5, MAX_WHEEL * 0.5)
                    bot.step()
                bot.stuck_counter = 0
                bot.path = []
                bot.a_star()
            
            dist_to_goal = math.hypot(bot.x - GOAL[0], bot.y - GOAL[1])
            if dist_to_goal < 0.05:
                print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
                bot.drive_wheels(0, 0)
                bot.state = "TRANSITION"
                
        elif bot.state == "TRANSITION":
            bot.camera.enable(TIME_STEP)
            bot.maze_start_x = 0.22
            bot.maze_start_y = 1.455
            
            # Step forward into Cell (0, 0) from the transition chamber
            dist_to_entry = 1.455 - bot.y
            if dist_to_entry > 0.02:
                bot.drive_wheels(MAX_WHEEL * 0.85, MAX_WHEEL * 0.85)
            else:
                bot.drive_wheels(0, 0)
                bot.x = 0.22
                bot.y = 1.455
                bot.th = math.pi / 2  # Facing North
                
                print("--- ENTERING SECTION 2 MAZE & STARTING FRONTIER EXPLORATION ---")
                bot.maze = MazeMap()
                bot.col = 0
                bot.row = 0
                bot.target_col = None
                bot.target_row = None
                bot.active_path = []
                bot.is_finishing = False
                
                bot.state = "S2_INSPECT"

        elif bot.state == "S2_INSPECT":
            # 0. Physical Odometry Correction: snap to true grid coordinates
            true_cx = bot.maze_start_x + bot.col * 0.25
            true_cy = bot.maze_start_y + bot.row * 0.25
            bot.x = true_cx
            bot.y = true_cy
            
            # Longitudinal correction against front wall if close
            if bot.d[0] < 0.20:
                dist_to_wall = bot.d[0] + 0.042
                if abs(wrap(bot.th - 0.0)) < 0.5: bot.x = (true_cx + 0.125) - dist_to_wall
                elif abs(wrap(bot.th - math.pi/2)) < 0.5: bot.y = (true_cy + 0.125) - dist_to_wall
                elif abs(wrap(bot.th - math.pi)) < 0.5: bot.x = (true_cx - 0.125) + dist_to_wall
                elif abs(wrap(bot.th + math.pi/2)) < 0.5: bot.y = (true_cy - 0.125) + dist_to_wall

            # 1. Camera Color Tile Detection
            detected_color = None
            img = bot.camera.getImageArray()
            if img:
                w, h = bot.camera.getWidth(), bot.camera.getHeight()
                r, g, b = img[w // 2][h // 2]
                if b > 40 and b > r * 1.5 and b > g * 1.5: detected_color = "BLUE"
                elif g > 40 and g > r * 1.5 and g > b * 1.5: detected_color = "GREEN"
                elif r > 40 and r > g * 1.5 and r > b * 1.5: detected_color = "RED"
            
            if detected_color:
                bot.maze.record_color(bot.col, bot.row, detected_color)

            # 2. Map Walls using distance sensors (first time inspecting this cell)
            cell = bot.maze.grid[(bot.col, bot.row)]
            if not cell.scanned:
                f_dir = angle_to_dir(bot.th)
                l_dir = angle_to_dir(bot.th + math.pi / 2)
                r_dir = angle_to_dir(bot.th - math.pi / 2)
                b_dir = angle_to_dir(bot.th + math.pi)
                
                bot.maze.set_edge(bot.col, bot.row, f_dir, "wall" if bot.d[0] < 0.16 else "open")
                bot.maze.set_edge(bot.col, bot.row, l_dir, "wall" if bot.d[2] < 0.16 else "open")
                bot.maze.set_edge(bot.col, bot.row, r_dir, "wall" if bot.d[6] < 0.16 else "open")
                bot.maze.set_edge(bot.col, bot.row, b_dir, "wall" if bot.d[4] < 0.16 else "open")
                
                cell.scanned = True
            cell.visited = True
            
            # 3. Plan next action immediately for continuous exploration
            print(f"Current cell: ({bot.col}, {bot.row})")
            bot.state = "S2_PLAN"

        elif bot.state == "S2_PLAN":
            # If we are in finish mode and finished the path back to the Red tile:
            if bot.is_finishing and not bot.active_path:
                bot.state = "S2_FINISH_ALIGN"
            
            # If we are currently following a multi-step path (backtracking or return to Red):
            elif bot.active_path:
                next_cell = bot.active_path.pop(0)
                dx = next_cell[0] - bot.col
                dy = next_cell[1] - bot.row
                d_str = {(0,1): "N", (0,-1): "S", (1,0): "E", (-1,0): "W"}.get((dx, dy), "")
                bot.target_col, bot.target_row = next_cell
                print(f"Next cell to go: {next_cell} ({d_str})")
                bot.state = "S2_TURN"
                
            else:
                # Find path to nearest unvisited cell via BFS
                frontier_path = bot.maze.find_nearest_unvisited_path(bot.col, bot.row)
                if frontier_path:
                    bot.active_path = frontier_path
                    next_cell = bot.active_path.pop(0)
                    dx = next_cell[0] - bot.col
                    dy = next_cell[1] - bot.row
                    d_str = {(0,1): "N", (0,-1): "S", (1,0): "E", (-1,0): "W"}.get((dx, dy), "")
                    bot.target_col, bot.target_row = next_cell
                    print(f"Next cell to go: {next_cell} ({d_str})")
                    bot.state = "S2_TURN"
                else:
                    # All reachable cells have been visited! 100% exploration complete!
                    print("--- All Reachable Cells Explored! Routing to Final Red Tile ---")
                    dest = bot.maze.red_cell_coord if bot.maze.red_cell_coord else (8, 3)
                    if (bot.col, bot.row) == dest:
                        bot.state = "S2_FINISH_ALIGN"
                    else:
                        route_to_red = bot.maze.find_path_to(bot.col, bot.row, dest[0], dest[1])
                        if route_to_red:
                            bot.active_path = route_to_red
                            bot.is_finishing = True
                            next_cell = bot.active_path.pop(0)
                            dx = next_cell[0] - bot.col
                            dy = next_cell[1] - bot.row
                            d_str = {(0,1): "N", (0,-1): "S", (1,0): "E", (-1,0): "W"}.get((dx, dy), "")
                            bot.target_col, bot.target_row = next_cell
                            print(f"Next cell to go: {next_cell} ({d_str})")
                            bot.state = "S2_TURN"
                        else:
                            print(f"[ERROR] Cannot find path to red cell {dest}!")
                            bot.state = "S2_FINISH_ALIGN"

        elif bot.state == "S2_TURN":
            dx = bot.target_col - bot.col
            dy = bot.target_row - bot.row
            target_angle = math.atan2(dy, dx)
            e = wrap(target_angle - bot.th)
            
            if abs(e) > 0.035:
                # Fast initial spin up to 70% MAX_WHEEL, smoothly ramps down to prevent overshoot
                speed = max(2.0, min(MAX_WHEEL * 0.70, abs(e) * 10.0))
                bot.drive_wheels(-speed if e > 0 else speed, speed if e > 0 else -speed)
            else:
                bot.state = "S2_MOVE"

        elif bot.state == "S2_MOVE":
            dx = bot.target_col - bot.col
            dy = bot.target_row - bot.row
            target_angle = math.atan2(dy, dx)

            target_x = bot.maze_start_x + bot.target_col * 0.25
            target_y = bot.maze_start_y + bot.target_row * 0.25
            
            remaining = (target_x - bot.x) * dx + (target_y - bot.y) * dy
            
            # 1. Arrival at target cell center
            if remaining <= 0.02:
                bot.x = target_x
                bot.y = target_y
                bot.col = bot.target_col
                bot.row = bot.target_row
                bot.state = "S2_INSPECT"
                
            # 2. Blocked by unexpected wall (only if far from arrival, > 0.08m)
            elif remaining > 0.08 and bot.d[0] < 0.05:
                bot.drive_wheels(0, 0)
                blocked_dir = {(0,1): "N", (0,-1): "S", (1,0): "E", (-1,0): "W"}.get((dx, dy), "N")
                print(f"[NAV] Blocked by unexpected wall heading {blocked_dir}! Re-planning at ({bot.col}, {bot.row})")
                bot.maze.set_edge(bot.col, bot.row, blocked_dir, "wall")
                bot.x = bot.maze_start_x + bot.col * 0.25
                bot.y = bot.maze_start_y + bot.row * 0.25
                bot.active_path = [] # Clear stale path
                bot.state = "S2_INSPECT"
                
            else:
                speed = MAX_WHEEL  # Full speed forward drive
                
                # 1. Primary Heading Lock using IMU
                heading_err = wrap(target_angle - bot.th)
                turn = heading_err * 3.5
                
                # 2. Parallel Wall Centering & Single-Wall Corner Repulsion
                left_wall = bot.d[2] < 0.16
                right_wall = bot.d[6] < 0.16
                if left_wall and right_wall:
                    wall_err = bot.d[2] - bot.d[6]
                    turn += wall_err * 2.0
                elif left_wall and not right_wall and bot.d[2] < 0.065:
                    # Too close to left wall: steer away to maintain corridor center
                    turn -= (0.075 - bot.d[2]) * 4.0
                elif right_wall and not left_wall and bot.d[6] < 0.065:
                    # Too close to right wall: steer away to maintain corridor center
                    turn += (0.075 - bot.d[6]) * 4.0
                    
                turn = max(min(turn, 3.0), -3.0)
                bot.drive_wheels(speed - turn, speed + turn)

        elif bot.state == "S2_FINISH_ALIGN":
            bot.drive_wheels(0, 0)
            dest = bot.maze.red_cell_coord if bot.maze.red_cell_coord else (8, 3)
            # Snap to Red cell center
            bot.x = bot.maze_start_x + dest[0] * 0.25
            bot.y = bot.maze_start_y + dest[1] * 0.25
            bot.col, bot.row = dest
            
            print("=== MAZE COMPLETED! ===")
            print(f"Final Count -> Blue: {bot.maze.blue_count}, Green: {bot.maze.green_count}")
            if bot.maze.blue_count > bot.maze.green_count:
                bot.led_blue.set(1)
                print("Majority Color: BLUE. Activated led_blue.")
            elif bot.maze.green_count > bot.maze.blue_count:
                bot.led_green.set(1)
                print("Majority Color: GREEN. Activated led_green.")
            else:
                bot.led_blue.set(1)
                bot.led_green.set(1)
                print("Color Tie! Activated both LEDs.")
            
            # Keep stepping briefly to allow Webots to render LED activation, then stop
            for _ in range(30):
                bot.step()
                bot.drive_wheels(0, 0)
            break

if __name__ == "__main__":
    main()
