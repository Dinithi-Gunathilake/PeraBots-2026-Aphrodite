"""
main_controller.py

PeraBots 2026 - Phase 1 - Complete Advanced SLAM
Step 4: A* Global Planning + 8-Sensor Occupancy Grid
"""

from controller import Robot
import math
import heapq

TIME_STEP = 16
MAX_WHEEL = 6.0
WHEEL_R = 0.02
ARENA_W, ARENA_H = 2.44, 1.22
GOAL = (0.22, 1.27)

CELL = 0.02
NX = int(round(ARENA_W / CELL))
NY = int(round(1.40 / CELL))
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
        self.map = bytearray(NX * NY)
        self.path = []

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
        """Project the 8 sensors into the Occupancy Grid."""
        for k, (_, sx, sy, ang) in enumerate(self.ds):
            d = self.d[k]
            if d >= 0.75: continue # Max range or inf
            
            abs_a = wrap(self.th + ang)
            px = self.x + (d + 0.042) * math.cos(abs_a)
            py = self.y + (d + 0.042) * math.sin(abs_a)
            
            ix = int(px / CELL)
            iy = int(py / CELL)
            
            if 0 <= ix < NX and 0 <= iy < NY:
                self.map[iy * NX + ix] = 1 # Mark as blocked
                
    def a_star(self):
        """Find fastest path to GOAL avoiding self.map obstacles."""
        start = (int(self.x / CELL), int(self.y / CELL))
        goal = (int(GOAL[0] / CELL), int(GOAL[1] / CELL))
        
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
                self.path = [((ix+0.5)*CELL, (iy+0.5)*CELL) for ix, iy in out]
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
                                if self.map[(nj+dy) * NX + (ni+dx)] == 1:
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
    bot.map = bytearray(NX * NY)
    print("--- Navigating Maze to Goal ---")
    
    plan_counter = 0
    bot.a_star() # Force A* immediately on step 0!
    
    while bot.step():
        plan_counter += 1
        
        # Re-plan A* path twice a second
        if plan_counter % 30 == 0:
            bot.a_star()
            
        bot.navigate()
        
        dist_to_goal = math.hypot(bot.x - GOAL[0], bot.y - GOAL[1])
        if dist_to_goal < 0.15:
            print("=== REACHED TRANSITION ZONE! ===")
            bot.drive_wheels(0, 0)
            break

if __name__ == "__main__":
    main()
