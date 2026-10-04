"""
main_controller.py

PeraBots 2026 - Phase 1 - Advanced SLAM
Step 2: Startup Auto-Calibration (RANSAC Line Fitting)
"""

from controller import Robot
import math

TIME_STEP = 16
MAX_WHEEL = 6.0

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def fit_wall(pts, centre, half):
    """Fit 1/r = A cos(phi) + B sin(phi) (a straight wall) to samples near 'centre'."""
    sel = [(p, 1.0 / r) for (p, r) in pts if abs(wrap(p - centre)) <= half]
    A = B = 0.0
    for _ in range(3):
        if len(sel) < 4:
            return None
        sxx = sxy = syy = sxu = syu = 0.0
        for p, u in sel:
            c, s = math.cos(p), math.sin(p)
            sxx += c * c; sxy += c * s; syy += s * s; sxu += c * u; syu += s * u
        det = sxx * syy - sxy * sxy
        if det < 1e-9:
            return None
        A = (sxu * syy - syu * sxy) / det
        B = (syu * sxx - sxu * sxy) / det
        keep = [(p, u) for (p, u) in sel
                if abs(A * math.cos(p) + B * math.sin(p) - u) < 0.08 * u]
        if len(keep) == len(sel):
            break
        sel = keep
    if math.hypot(A, B) < 1e-6:
        return None
    return math.atan2(B, A), 1.0 / math.hypot(A, B)

def predict_reading(x0, y0, alpha):
    ca, sa = math.cos(alpha), math.sin(alpha)
    t = 1e18
    # Arena bounds: 2.44 x 1.22. Wall thickness: 0.006
    if ca > 1e-9: t = min(t, (2.44 - 0.006 - x0) / ca)
    if ca < -1e-9: t = min(t, (0.006 - x0) / ca)
    if sa > 1e-9: t = min(t, (1.22 - 0.006 - y0) / sa)
    if sa < -1e-9: t = min(t, (0.006 - y0) / sa)
    return min(0.8, t - 0.042)

def localise_from_scan(samples):
    pts = [(p, d + 0.042) for (p, d) in samples if d < 0.75]
    if len(pts) < 20:
        return None
    pm, _ = min(pts, key=lambda q: q[1])
    w1 = fit_wall(pts, pm, 0.35)
    if w1 is None:
        return None
    a1, D1 = w1
    best = None
    for sgn in (-1, 1):
        c = wrap(a1 + sgn * math.pi / 2)
        w = fit_wall(pts, c, 0.21)
        if w is not None and abs(wrap(w[0] - c)) < 0.15:
            if best is None or w[1] < best[1][1]:
                best = (sgn, w)
    if best is None:
        return None
    sgn, (a2, D2) = best
    if sgn == -1:
        phi_x, Dx, Dy = a1, D1, D2
    else:
        phi_x, Dx, Dy = a2, D2, D1
        
    x0 = 2.44 - 0.006 - Dx
    y0 = 0.006 + Dy
    return x0, y0, phi_x

def main():
    robot = Robot()
    
    # 1. Initialize Motors
    left_motor = robot.getDevice('left_wheel_motor')
    right_motor = robot.getDevice('right_wheel_motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    # 2. Initialize Sensors (Only need front and IMU for spinning calib)
    ds_front = robot.getDevice('ds_front')
    ds_front.enable(TIME_STEP)
    
    imu = robot.getDevice('inertial_unit')
    imu.enable(TIME_STEP)
    
    print("--- Starting Auto-Calibration Spin ---")
    
    # Spin in place
    left_motor.setVelocity(MAX_WHEEL)
    right_motor.setVelocity(-MAX_WHEEL)
    
    samples = []
    
    # Collect data for about 3.5 seconds (220 steps)
    for _ in range(220):
        if robot.step(TIME_STEP) == -1:
            return
            
        yaw = imu.getRollPitchYaw()[2]
        dist = ds_front.getValue()
        
        # In case it's hitting the limit
        if dist != dist:
            dist = 0.8
            
        samples.append((yaw, dist))
        
    # Stop spinning
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    print(f"Collected {len(samples)} data points. Calculating exact position...")
    
    # Calculate position
    result = localise_from_scan(samples)
    
    if result:
        x, y, theta = result
        print("=========================================")
        print("  AUTO-CALIBRATION SUCCESSFUL!  ")
        print(f"  Calculated X: {x:.4f} m")
        print(f"  Calculated Y: {y:.4f} m")
        print(f"  Calculated Theta: {theta:.4f} rad")
        print("=========================================")
    else:
        print("Calibration failed. Start zone walls not found.")

if __name__ == "__main__":
    main()
