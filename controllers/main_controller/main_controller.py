"""
main_controller.py

PeraBots 2026 - Phase 1 - Complete Advanced SLAM
Step 4: A* Global Planning + 8-Sensor Occupancy Grid
"""

from controller import Robot
import math
import heapq
import collections
import sys
import os

class Tee:
    def __init__(self, filepath):
        self.file = open(filepath, "w", buffering=1, encoding="utf-8")
        self.stdout = sys.stdout
    def write(self, data):
        self.stdout.write(data)
        self.file.write(data)
    def flush(self):
        self.stdout.flush()
        self.file.flush()

log_path = os.path.join(os.path.dirname(__file__), "run.log")
sys.stdout = Tee(log_path)
sys.stderr = sys.stdout

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
        self.path = []
        
        # State Machine & Hardware
        self.state = "SECTION_1"
        self.camera = robot.getDevice("camera")
        self.camera.enable(TIME_STEP)
        self.led_blue = robot.getDevice("led_blue")
        self.led_green = robot.getDevice("led_green")
        
        # Color Counting & Memory
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = set()
        self.final_end_cell = None
        self.step_count = 0
        self.cx = 0
        self.cy = 0
        self.target_cx = 0
        self.target_cy = 0

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
        
        if self.state == "SECTION_1":
            self.update_map()
        elif self.state in ["DFS_SCAN", "DFS_TURN", "DFS_FORWARD", "FINISH_NAV"]:
            self.check_floor_tile()
        
        self.step_count += 1
        return True

    def check_floor_tile(self):
        img = self.camera.getImageArray()
        if not img: return
        w, h = self.camera.getWidth(), self.camera.getHeight()
        r_sum = g_sum = b_sum = 0
        cnt = 0
        for px_i in range(w // 2 - 3, w // 2 + 4):
            for px_j in range(h // 2 - 3, h // 2 + 4):
                pr, pg, pb = img[px_i][px_j]
                r_sum += pr
                g_sum += pg
                b_sum += pb
                cnt += 1
        r = r_sum / cnt
        g = g_sum / cnt
        b = b_sum / cnt
        
        detected = None
        if b > 28 and b > r * 1.3 and b > g * 1.3:
            detected = "BLUE"
        elif g > 28 and g > r * 1.3 and g > b * 1.3:
            detected = "GREEN"
        elif r > 35 and r > g * 1.3 and r > b * 1.3:
            detected = "RED"
            
        if detected:
            cell_x = int(round((self.x - 0.22) / 0.25))
            cell_y = int(round((self.y - 1.455) / 0.25))
            if 0 <= cell_x <= 8 and 0 <= cell_y <= 3 and (cell_x, cell_y) not in self.tile_memory:
                self.tile_memory.add((cell_x, cell_y))
                if detected == "BLUE":
                    self.blue_count += 1
                    print(f"[CAMERA] Found BLUE tile at ({cell_x}, {cell_y})! Total Blue: {self.blue_count}")
                elif detected == "GREEN":
                    self.green_count += 1
                    print(f"[CAMERA] Found GREEN tile at ({cell_x}, {cell_y})! Total Green: {self.green_count}")
                elif detected == "RED" and (cell_x > 0 or cell_y > 0):
                    self.final_end_cell = (cell_x, cell_y)
                    print(f"[CAMERA] Found FINAL RED TILE at ({cell_x}, {cell_y})!")

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
                            
        # 2. Sensor Raycasting with exact mounting position transforms
        cos_th = math.cos(self.th)
        sin_th = math.sin(self.th)
        
        for k, (_, sx, sy, ang) in enumerate(self.ds):
            d = self.d[k]
            abs_a = wrap(self.th + ang)
            
            # True sensor world position
            wsx = self.x + sx * cos_th - sy * sin_th
            wsy = self.y + sx * sin_th + sy * cos_th
            
            # Max range for raycast clearing
            clear_d = min(d, 0.75)
            
            # Bresenham to clear free space
            px_clear = wsx + clear_d * math.cos(abs_a)
            py_clear = wsy + clear_d * math.sin(abs_a)
            
            x0 = int(wsx / CELL) + OFFSET_X
            y0 = int(wsy / CELL) + OFFSET_Y
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
                    
            # Mark Obstacle at measured physical reflection point
            if d < 0.75:
                px = wsx + d * math.cos(abs_a)
                py = wsy + d * math.sin(abs_a)
                ix = int(px / CELL) + OFFSET_X
                iy = int(py / CELL) + OFFSET_Y
                if 0 <= ix < NX and 0 <= iy < NY:
                    self.map[iy * NX + ix] = 255

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
                            
                            # Apply Obstacle Inflation check!
                            blocked = False
                            for idx in [-2, -1, 0, 1, 2]:
                                for idy in [-2, -1, 0, 1, 2]:
                                    if idx*idx + idy*idy <= 5: # 4cm circular clearance
                                        ix, iy = nx+idx, ny+idy
                                        if 0 <= ix < NX and 0 <= iy < NY:
                                            if self.map[iy * NX + ix] == 255:
                                                blocked = True
                                                break
                                if blocked: break
                                
                            if not blocked:
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
        g = [INF] * N
        parent = [-1] * N
        s_idx = start[1] * NX + start[0]
        g[s_idx] = 0.0
        heap = [(0.0, s_idx)]
        closed = bytearray(N)
        
        moves = [(1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0), 
                 (1,1,1.414), (1,-1,1.414), (-1,1,1.414), (-1,-1,1.414)]
                 
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
                
                # Check obstacle + circular inflation (radius = 2 cells = 4cm clearance)
                blocked = False
                for dx in [-2, -1, 0, 1, 2]:
                    for dy in [-2, -1, 0, 1, 2]:
                        if dx*dx + dy*dy <= 5: # Circular footprint radius ~2.2
                            if 0 <= ni+dx < NX and 0 <= nj+dy < NY:
                                if self.map[(nj+dy) * NX + (ni+dx)] == 255:
                                    blocked = True
                
                nidx = nj * NX + ni
                if blocked or closed[nidx]: continue
                
                ng = g[idx] + cost
                if ng < g[nidx]:
                    g[nidx] = ng
                    parent[nidx] = idx
                    # Heuristic
                    h = math.hypot(ni - goal[0], nj - goal[1])
                    heapq.heappush(heap, (ng + h, nidx))
                    
        print(f"[A* ERROR] Queue emptied. No path found to GOAL {goal} from START {start}!")
        self.path = []

    def navigate(self):
        """Pure Pursuit Follower to drive along self.path."""
        if not self.path or len(self.path) < 2:
            print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
            self.drive_wheels(0, 0)
            return
            
        # Find closest point on path to current robot position
        min_dist = INF
        closest_idx = 0
        for i, p in enumerate(self.path):
            d = math.hypot(p[0] - self.x, p[1] - self.y)
            if d < min_dist:
                min_dist = d
                closest_idx = i
                
        # Lookahead 10 nodes (20cm) to prevent corner cutting
        lookahead = min(closest_idx + 10, len(self.path) - 1)
        target = self.path[lookahead]
        
        # Calculate heading error
        target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
        e = wrap(target_angle - self.th)
        
        # Drive control - Smoother!
        if abs(e) > 0.6: # Sharp turn: spin in place
            v_l = MAX_WHEEL * 0.5 if e < 0 else -MAX_WHEEL * 0.5
            v_r = -MAX_WHEEL * 0.5 if e < 0 else MAX_WHEEL * 0.5
        else: # Smooth proportional curve
            speed = MAX_WHEEL * 0.8  # slower base speed
            turn = e * MAX_WHEEL * 0.8  # much gentler steering gain
            
            # Repulsive Steering: Push away from close corners to prevent clipping!
            if self.d[1] < 0.12: # Front-Left sees corner
                turn -= (0.12 - self.d[1]) * MAX_WHEEL * 3.0
            if self.d[7] < 0.12: # Front-Right sees corner
                turn += (0.12 - self.d[7]) * MAX_WHEEL * 3.0
            
            # Slow down if obstacle is close in front
            if min(self.d[0], self.d[1], self.d[7]) < 0.2:
                speed *= 0.5
                
            v_l = speed - turn
            v_r = speed + turn
        self.drive_wheels(v_l, v_r)
def main():
    global GOAL
    robot = Robot()
    bot = Bot(robot)
    print("--- Starting Auto-Calibration Spin ---")
    bot.drive_wheels(MAX_WHEEL, -MAX_WHEEL)
    
    samples = []
    for _ in range(220):
        if not bot.step(): return
        samples.append((bot.th, bot.d[0]))
        
    bot.drive_wheels(0, 0)
    
    result = localise_from_scan(samples)
    if result:
        bot.x, bot.y, _ = result
        print(f"Calibrated: X={bot.x:.3f}, Y={bot.y:.3f}")
    else:
        print("Calibration failed!")
        return

    # Clear map garbage collected during the spin when coordinates were 0,0
    bot.map = bytearray([127] * (NX * NY))
    print("--- Navigating Maze to Goal ---")
    
    plan_counter = 0
    bot.a_star() # Force A* immediately on step 0!
    
    while bot.step():
        if bot.step_count % 150 == 0 and bot.state not in ["SECTION_1", "TRANSITION"]:
            print(f"[STATUS {bot.step_count}] state={bot.state} cell=({bot.cx},{bot.cy})->({bot.target_cx},{bot.target_cy}) pos=({bot.x:.2f},{bot.y:.2f}) th={bot.th:.2f}")

        if bot.state == "SECTION_1":
            plan_counter += 1
            if plan_counter % 30 == 0:
                bot.a_star()
            bot.navigate()
            
            dist_to_goal = math.hypot(bot.x - GOAL[0], bot.y - GOAL[1])
            if dist_to_goal < 0.05:
                print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
                bot.drive_wheels(0, 0)
                bot.state = "TRANSITION"
                
        elif bot.state == "TRANSITION":
            # 1. Align to face North (math.pi / 2) before driving through narrow corridor
            heading_err = wrap(math.pi / 2 - bot.th)
            if abs(heading_err) > 0.05:
                turn = max(-3.0, min(3.0, 3.5 * heading_err))
                if abs(turn) < 0.8: turn = 0.8 if turn > 0 else -0.8
                bot.drive_wheels(-turn, turn)
            elif bot.y < 1.44:
                # Drive forward through chamber into cell (0, 0)
                speed = 4.0
                turn = heading_err * 3.0
                if bot.d[2] < 0.15 and bot.d[6] < 0.15:
                    turn += (bot.d[2] - bot.d[6]) * 2.0
                turn = max(-2.0, min(2.0, turn))
                bot.drive_wheels(speed - turn, speed + turn)
            else:
                bot.drive_wheels(0, 0)
                bot.x = 0.22
                bot.y = 1.455
                bot.cx = 0
                bot.cy = 0
                bot.maze_walls = {}
                bot.dfs_visited = {(0, 0)}
                bot.dfs_stack = []
                bot.tile_memory = set()
                bot.is_finishing = False
                print("[TRANSITION] Arrived at Cell (0, 0). Starting Maze Exploration!")
                bot.state = "DFS_SCAN"
            
        elif bot.state == "DFS_SCAN":
            # 0. Check camera for floor tile color with window average
            img = bot.camera.getImageArray()
            if img:
                w, h = bot.camera.getWidth(), bot.camera.getHeight()
                r_sum = g_sum = b_sum = 0
                pix_count = 0
                for px_i in range(max(0, w // 2 - 3), min(w, w // 2 + 4)):
                    for px_j in range(max(0, h // 2 - 3), min(h, h // 2 + 4)):
                        pr, pg, pb = img[px_i][px_j]
                        r_sum += pr
                        g_sum += pg
                        b_sum += pb
                        pix_count += 1
                r = r_sum / pix_count
                g = g_sum / pix_count
                b = b_sum / pix_count
                print(f"[CAM DEBUG] ({bot.cx},{bot.cy}): R={r:.1f}, G={g:.1f}, B={b:.1f}")
                
                detected_color = None
                if b > 28 and b > r * 1.25 and b > g * 1.25:
                    detected_color = "BLUE"
                elif g > 28 and g > r * 1.25 and g > b * 1.25:
                    detected_color = "GREEN"
                elif r > 28 and r > g * 1.25 and r > b * 1.25:
                    detected_color = "RED"
                    
                if detected_color and (bot.cx, bot.cy) not in bot.tile_memory:
                    bot.tile_memory.add((bot.cx, bot.cy))
                    if detected_color == "BLUE":
                        bot.blue_count += 1
                        print(f"[CAMERA] Found BLUE tile at ({bot.cx}, {bot.cy})! Total Blue: {bot.blue_count}")
                    elif detected_color == "GREEN":
                        bot.green_count += 1
                        print(f"[CAMERA] Found GREEN tile at ({bot.cx}, {bot.cy})! Total Green: {bot.green_count}")
                    elif detected_color == "RED" and (bot.cx > 0 or bot.cy > 0):
                        bot.final_end_cell = (bot.cx, bot.cy)
                        print(f"[CAMERA] Found FINAL RED TILE at ({bot.cx}, {bot.cy})!")

            # 1. Align to exact cardinal heading before reading distance sensors
            th_w = wrap(bot.th)
            if abs(th_w - 0.0) < 0.78:
                cardinal_th = 0.0
                f_dir, l_dir, r_dir, b_dir = "E", "N", "S", "W"
            elif abs(wrap(th_w - math.pi / 2)) < 0.78:
                cardinal_th = math.pi / 2
                f_dir, l_dir, r_dir, b_dir = "N", "W", "E", "S"
            elif abs(wrap(th_w - math.pi)) < 0.78:
                cardinal_th = math.pi
                f_dir, l_dir, r_dir, b_dir = "W", "S", "N", "E"
            else:
                cardinal_th = -math.pi / 2
                f_dir, l_dir, r_dir, b_dir = "S", "E", "W", "N"

            align_err = wrap(cardinal_th - bot.th)
            if abs(align_err) > 0.04:
                turn = max(-2.0, min(2.0, 3.5 * align_err))
                if abs(turn) < 0.8: turn = 0.8 if turn > 0 else -0.8
                bot.drive_wheels(-turn, turn)
                continue
            bot.drive_wheels(0, 0)

            # 2. Map walls for current cell if not already mapped
            if (bot.cx, bot.cy) not in bot.maze_walls:
                WALL_THRESH = 0.125
                walls = {
                    f_dir: (bot.d[0] < WALL_THRESH),
                    l_dir: (bot.d[2] < WALL_THRESH),
                    r_dir: (bot.d[6] < WALL_THRESH),
                    b_dir: False  # The corridor we entered from is open
                }
                
                # Outer perimeter boundaries of 9x4 maze
                if bot.cy == 3: walls["N"] = True
                if bot.cy == 0: walls["S"] = True
                if bot.cx == 0: walls["W"] = True
                if bot.cx == 8: walls["E"] = True
                
                # Entrance door from Section 1 is closed
                if bot.cx == 0 and bot.cy == 0:
                    walls["S"] = True
                    
                bot.maze_walls[(bot.cx, bot.cy)] = walls
                print(f"[WALLS DEBUG] ({bot.cx}, {bot.cy}) pos=({bot.x:.3f}, {bot.y:.3f}) th={bot.th:.2f} dirs=({f_dir},{l_dir},{r_dir},{b_dir}) d0={bot.d[0]:.3f} d2={bot.d[2]:.3f} d6={bot.d[6]:.3f} -> walls={walls}")
                
                # Bidirectional consistency: confirmed open passages remain open
                dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
                opp_dir = {"N": "S", "S": "N", "E": "W", "W": "E"}
                for d_name, has_wall in walls.items():
                    dx, dy = dir_deltas[d_name]
                    nx, ny = bot.cx + dx, bot.cy + dy
                    if 0 <= nx <= 8 and 0 <= ny <= 3 and (nx, ny) in bot.maze_walls:
                        if not has_wall:
                            bot.maze_walls[(nx, ny)][opp_dir[d_name]] = False

            # 2. Pick next cell via DFS
            dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
            walls = bot.maze_walls[(bot.cx, bot.cy)]
            unvisited = []
            for d_name in ["N", "E", "S", "W"]:
                if not walls.get(d_name, True):
                    dx, dy = dir_deltas[d_name]
                    nx, ny = bot.cx + dx, bot.cy + dy
                    if 0 <= nx <= 8 and 0 <= ny <= 3:
                        if (nx, ny) not in bot.dfs_visited:
                            unvisited.append(((nx, ny), d_name))
                            
            if unvisited:
                next_cell, d_name = unvisited[0]
                bot.dfs_visited.add(next_cell)
                bot.dfs_stack.append((bot.cx, bot.cy))
                bot.target_cx, bot.target_cy = next_cell
                print(f"[DFS] At ({bot.cx}, {bot.cy}) -> Move to unvisited {next_cell} ({d_name})")
                bot.state = "DFS_TURN"
            else:
                if not bot.dfs_stack:
                    print("=== DFS COMPLETE! All reachable cells visited. ===")
                    bot.state = "FINISH"
                else:
                    back_cell = bot.dfs_stack.pop()
                    bot.target_cx, bot.target_cy = back_cell
                    print(f"[DFS] At ({bot.cx}, {bot.cy}) [Dead End] -> Backtrack to {back_cell}")
                    bot.state = "DFS_TURN"
                    
        elif bot.state == "DFS_TURN":
            # If closer than 0.070m to front wall and rear is open, back up slightly for clearance
            if bot.d[0] < 0.070 and bot.d[4] > 0.09:
                bot.drive_wheels(-2.0, -2.0)
            else:
                if abs(bot.target_cx - bot.cx) > 0:
                    target_angle = 0.0 if bot.target_cx > bot.cx else math.pi
                else:
                    target_angle = math.pi / 2 if bot.target_cy > bot.cy else -math.pi / 2
                    
                e = wrap(target_angle - bot.th)
                if abs(e) > 0.08:
                    turn = max(-2.5, min(2.5, 3.5 * e))
                    if abs(turn) < 1.0: turn = 1.0 if turn > 0 else -1.0
                    bot.drive_wheels(-turn, turn)
                else:
                    bot.drive_wheels(0, 0)
                    bot.state = "DFS_FORWARD"
                
        elif bot.state == "DFS_FORWARD":
            target_x = 0.22 + bot.target_cx * 0.25
            target_y = 1.455 + bot.target_cy * 0.25
            
            # Progress calculation along motion axis
            if bot.target_cx > bot.cx:
                rem = target_x - bot.x
            elif bot.target_cx < bot.cx:
                rem = bot.x - target_x
            elif bot.target_cy > bot.cy:
                rem = target_y - bot.y
            else:
                rem = bot.y - target_y

            has_wall_ahead = (bot.d[0] < 0.16)
            arrived = (rem <= 0.015) or (rem <= 0.035 and has_wall_ahead and bot.d[0] <= 0.080)
            
            if arrived:
                print(f"[ARRIVED] At ({bot.target_cx}, {bot.target_cy}) step={bot.step_count} rem={rem:.3f} d0={bot.d[0]:.3f} actual_pos=({bot.x:.3f}, {bot.y:.3f})")
                bot.drive_wheels(0, 0)
                bot.cx = bot.target_cx
                bot.cy = bot.target_cy
                bot.x = target_x
                bot.y = target_y
                bot.state = "FINISH_NAV" if bot.is_finishing else "DFS_SCAN"
            else:
                speed = 4.0
                if abs(bot.target_cx - bot.cx) > 0:
                    nominal_y = 1.455 + bot.target_cy * 0.25
                    base_angle = 0.0 if bot.target_cx > bot.cx else math.pi
                    lat_err = nominal_y - bot.y
                    correction = max(-0.35, min(0.35, lat_err * 4.0))
                    target_angle = wrap(base_angle + (correction if base_angle == 0.0 else -correction))
                else:
                    nominal_x = 0.22 + bot.target_cx * 0.25
                    base_angle = math.pi / 2 if bot.target_cy > bot.cy else -math.pi / 2
                    lat_err = nominal_x - bot.x
                    correction = max(-0.35, min(0.35, lat_err * 4.0))
                    target_angle = wrap(base_angle - (correction if base_angle > 0 else -correction))
                    
                heading_err = wrap(target_angle - bot.th)
                turn = heading_err * 4.0
                
                # Dynamic wall clearance & corridor centering
                if bot.d[2] < 0.14 and bot.d[6] < 0.14:
                    turn += (bot.d[2] - bot.d[6]) * 3.0
                elif bot.d[2] < 0.11:
                    turn += (bot.d[2] - 0.083) * 3.0
                elif bot.d[6] < 0.11:
                    turn += (0.083 - bot.d[6]) * 3.0
                    
                turn = max(-2.0, min(2.0, turn))
                bot.drive_wheels(speed - turn, speed + turn)
                
        elif bot.state == "FINISH":
            if not bot.final_end_cell:
                print("[WARN] Red tile not seen visually. Defaulting to (8, 3).")
                bot.final_end_cell = (8, 3)
                
            print(f"[FINISH] Planning shortest path to End Cell {bot.final_end_cell}...")
            dir_deltas = {"N": (0, 1), "S": (0, -1), "E": (1, 0), "W": (-1, 0)}
            queue = collections.deque([((bot.cx, bot.cy), [])])
            visited = {(bot.cx, bot.cy)}
            bot.discrete_path = []
            while queue:
                curr, path = queue.popleft()
                if curr == bot.final_end_cell:
                    bot.discrete_path = path
                    break
                walls = bot.maze_walls.get(curr, {})
                for d_name, (dx, dy) in dir_deltas.items():
                    if not walls.get(d_name, True):
                        nxt = (curr[0] + dx, curr[1] + dy)
                        if 0 <= nxt[0] <= 8 and 0 <= nxt[1] <= 3 and nxt not in visited:
                            visited.add(nxt)
                            queue.append((nxt, path + [nxt]))
                            
            if not bot.discrete_path:
                if (bot.cx, bot.cy) == bot.final_end_cell:
                    print("[FINISH] Already at end cell!")
                    bot.state = "FINISH_NAV"
                else:
                    print(f"[FINISH WARNING] Fallback BFS to reach {bot.final_end_cell}...")
                    queue = collections.deque([((bot.cx, bot.cy), [])])
                    visited = {(bot.cx, bot.cy)}
                    while queue:
                        curr, path = queue.popleft()
                        if curr == bot.final_end_cell:
                            bot.discrete_path = path
                            break
                        walls = bot.maze_walls.get(curr, {})
                        for d_name, (dx, dy) in dir_deltas.items():
                            if not walls.get(d_name, False): # Treat unknown as passable in fallback
                                nxt = (curr[0] + dx, curr[1] + dy)
                                if 0 <= nxt[0] <= 8 and 0 <= nxt[1] <= 3 and nxt not in visited:
                                    visited.add(nxt)
                                    queue.append((nxt, path + [nxt]))
                    print(f"[FINISH] Fallback path: {bot.discrete_path}")
                    bot.is_finishing = True
                    bot.state = "FINISH_NAV"
            else:
                print(f"[FINISH] Path to end cell: {bot.discrete_path}")
                bot.is_finishing = True
                bot.state = "FINISH_NAV"
                
        elif bot.state == "FINISH_NAV":
            if not bot.discrete_path:
                bot.drive_wheels(0, 0)
                print("========================================")
                print("===      CHALLENGE COMPLETED!        ===")
                print(f"=== Total Blue: {bot.blue_count}, Total Green: {bot.green_count} ===")
                if bot.blue_count > bot.green_count:
                    print("=== Majority: BLUE -> Turning on Blue LED ===")
                    bot.led_blue.set(1)
                elif bot.green_count > bot.blue_count:
                    print("=== Majority: GREEN -> Turning on Green LED ===")
                    bot.led_green.set(1)
                else:
                    print("=== Tie -> Turning on Blue LED ===")
                    bot.led_blue.set(1)
                print("========================================")
                while bot.step():
                    bot.drive_wheels(0, 0)
                break
                
            bot.target_cx, bot.target_cy = bot.discrete_path.pop(0)
            bot.state = "DFS_TURN"

if __name__ == "__main__":
    main()
