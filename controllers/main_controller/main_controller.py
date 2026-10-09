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
MAX_WHEEL = 6.0
WHEEL_R = 0.02
ARENA_W, ARENA_H = 2.44, 1.22
GOAL = (0.22, 1.27)

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
        self.led_blue = robot.getDevice("led_blue")
        self.led_green = robot.getDevice("led_green")
        
        # Color Counting & Memory
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = []
        self.final_end_cell = None

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
        
        self.scan_match()
        self.update_map()
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
            print(f"[NAVIGATE] Stuck! Path length is {len(self.path) if self.path else 0}. Motors OFF.")
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
            
        print(f"[NAVIGATE] Path: {len(self.path)} nodes. e: {math.degrees(e):.1f}°. v_l: {v_l:.2f}, v_r: {v_r:.2f}")
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
        if bot.state == "SECTION_1":
            plan_counter += 1
            if plan_counter % 30 == 0:
                bot.a_star()
            bot.navigate()
            
            dist_to_goal = math.hypot(bot.x - GOAL[0], bot.y - GOAL[1])
            if dist_to_goal < 0.15:
                print("=== REACHED TRANSITION ZONE! ===")
                bot.drive_wheels(0, 0)
                bot.state = "TRANSITION"
                
        elif bot.state == "TRANSITION":
            print("--- ACTIVATING CAMERA & CLEARING MAP FOR MAZE ---")
            bot.camera.enable(TIME_STEP)
            bot.map = bytearray([127] * (NX * NY)) # Clear Section 1 obstacles, reset to Unknown
            bot.path = []
            bot.state = "EXPLORE"
            
        elif bot.state == "EXPLORE":
            plan_counter += 1
            if plan_counter % 20 == 0 or not bot.path:
                bot.plan_to_frontier()
                
                # If no frontiers left, we mapped everything! Time to finish.
                if not bot.path:
                    bot.state = "FINISH"
                    print("--- EXPLORATION COMPLETE. TRANSITIONING TO FINISH ---")
                    continue
                    
            bot.navigate()
            
            # Step 4: Color Detection & Counting
            img = bot.camera.getImageArray()
            if img:
                # Sample the center pixel
                w, h = bot.camera.getWidth(), bot.camera.getHeight()
                r, g, b = img[w // 2][h // 2]
                
                detected_color = None
                # Basic RGB Thresholding
                if b > 100 and b > r * 1.5 and b > g * 1.5:
                    detected_color = "BLUE"
                elif g > 100 and g > r * 1.5 and g > b * 1.5:
                    detected_color = "GREEN"
                elif r > 100 and r > g * 1.5 and r > b * 1.5:
                    detected_color = "RED"
                    
                if detected_color:
                    # Spatial Memory: Only count if it's > 15cm away from known tiles
                    is_new = True
                    for (mx, my, mcolor) in bot.tile_memory:
                        if math.hypot(bot.x - mx, bot.y - my) < 0.15:
                            is_new = False
                            break
                            
                    if is_new:
                        bot.tile_memory.append((bot.x, bot.y, detected_color))
                        if detected_color == "BLUE":
                            bot.blue_count += 1
                            print(f"[CAMERA] Found BLUE tile! Total Blue: {bot.blue_count}")
                        elif detected_color == "GREEN":
                            bot.green_count += 1
                            print(f"[CAMERA] Found GREEN tile! Total Green: {bot.green_count}")
                        elif detected_color == "RED":
                            # Check if it's the final end cell (must be far away from transition zone!)
                            if math.hypot(bot.x - GOAL[0], bot.y - GOAL[1]) > 0.50:
                                bot.final_end_cell = (bot.x, bot.y)
                                print(f"[CAMERA] Memorized FINAL RED TILE at {bot.final_end_cell}!")
            
        elif bot.state == "FINISH":
            if not bot.final_end_cell:
                print("[ERROR] Explored whole maze but never found the final Red tile!")
                bot.drive_wheels(0, 0)
                break
                
            # Route back to the Final Red Tile
            if not bot.path:
                old_goal = GOAL
                GOAL = bot.final_end_cell
                bot.a_star()
                GOAL = old_goal
                
            bot.navigate()
            
            # Check if we reached it
            if math.hypot(bot.x - bot.final_end_cell[0], bot.y - bot.final_end_cell[1]) < 0.10:
                print("=== MAZE COMPLETED! ===")
                print(f"Final Count -> Blue: {bot.blue_count}, Green: {bot.green_count}")
                bot.drive_wheels(0, 0)
                
                # Illuminate corresponding LED
                if bot.blue_count > bot.green_count:
                    bot.led_blue.set(1)
                    print("Lighting BLUE LED!")
                elif bot.green_count > bot.blue_count:
                    bot.led_green.set(1)
                    print("Lighting GREEN LED!")
                else:
                    print("Tie! Lighting both!")
                    bot.led_blue.set(1)
                    bot.led_green.set(1)
                    
                break # Challenge Finished!

if __name__ == "__main__":
    main()
