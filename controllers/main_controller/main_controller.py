"""
main_controller.py

PeraBots 2026 - Phase 1 - Advanced SLAM
Step 3: Drift-Free Scan Matching + Odometry
"""

from controller import Robot
import math

TIME_STEP = 16
MAX_WHEEL = 6.0
WHEEL_R = 0.02
ARENA_W, ARENA_H = 2.44, 1.22

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

    def drive_wheels(self, wl, wr):
        self.lm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wl)))
        self.rm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wr)))

    def step(self):
        if self.robot.step(TIME_STEP) == -1: return False
        
        # Read Sensors
        for k, (dev, _, _, _) in enumerate(self.ds):
            v = dev.getValue()
            self.d[k] = 0.8 if v != v else max(0.0, min(0.8, v))
            
        # Read IMU for Drift-Free Heading
        self.th = self.imu.getRollPitchYaw()[2]
        
        # Update Odometry for X and Y
        wl, wr = self.ls.getValue(), self.rs.getValue()
        if self.wl_prev is not None:
            ds = 0.5 * ((wl - self.wl_prev) + (wr - self.wr_prev)) * WHEEL_R
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        self.wl_prev, self.wr_prev = wl, wr
        
        self.scan_match()
        return True

    def scan_match(self):
        """Corrects wheel slip by comparing our 8 sensors against the fixed outer walls."""
        for k, (_, _, _, ang) in enumerate(self.ds):
            d = self.d[k]
            if d >= 0.7: continue # Ignore far readings
                
            abs_angle = wrap(self.th + ang)
            
            # If looking +X (Right Wall)
            if abs(wrap(abs_angle - 0.0)) < 0.08:
                pred_x = ARENA_W - 0.006 - d - 0.042
                if abs(pred_x - self.x) < 0.15: # Must be wall, not an obstacle cube
                    self.x = 0.90 * self.x + 0.10 * pred_x
                    
            # If looking +Y (Top Wall)
            elif abs(wrap(abs_angle - (math.pi/2))) < 0.08:
                pred_y = ARENA_H - 0.006 - d - 0.042
                if abs(pred_y - self.y) < 0.15:
                    self.y = 0.90 * self.y + 0.10 * pred_y
                    
            # If looking -X (Left Wall)
            elif abs(wrap(abs_angle - math.pi)) < 0.08:
                pred_x = 0.006 + d + 0.042
                if abs(pred_x - self.x) < 0.15:
                    self.x = 0.90 * self.x + 0.10 * pred_x
                    
            # If looking -Y (Bottom Wall)
            elif abs(wrap(abs_angle - (-math.pi/2))) < 0.08:
                pred_y = 0.006 + d + 0.042
                if abs(pred_y - self.y) < 0.15:
                    self.y = 0.90 * self.y + 0.10 * pred_y

def main():
    robot = Robot()
    bot = Bot(robot)
    
    print("--- Starting Auto-Calibration Spin ---")
    bot.drive_wheels(MAX_WHEEL, -MAX_WHEEL)
    
    samples = []
    for _ in range(220):
        if not bot.step(): return
        samples.append((bot.th, bot.d[0])) # Yaw and ds_front
        
    bot.drive_wheels(0, 0)
    
    result = localise_from_scan(samples)
    if result:
        bot.x, bot.y, _ = result
        print(f"Calibrated: X={bot.x:.3f}, Y={bot.y:.3f}")
    else:
        print("Calibration failed!")
        return

    print("--- Testing Drift-Free Odometry ---")
    # Drive forward slightly to test the scan-matching
    bot.drive_wheels(MAX_WHEEL, MAX_WHEEL)
    
    counter = 0
    while bot.step():
        counter += 1
        if counter % 30 == 0:
            print(f"[LIVE MAP] X: {bot.x:.3f} m, Y: {bot.y:.3f} m, Head: {math.degrees(bot.th):.1f}°")

if __name__ == "__main__":
    main()
