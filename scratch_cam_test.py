from controller import Robot

robot = Robot()
camera = robot.getDevice("camera")
camera.enable(32)

for i in range(10):
    robot.step(32)

img = camera.getImageArray()
w = camera.getWidth()
h = camera.getHeight()
print(f"Camera size: {w}x{h}")
r_mid, g_mid, b_mid = img[w//2][h//2]
print(f"Center pixel at start (red marker): R={r_mid}, G={g_mid}, B={b_mid}")

# Let's inspect all pixels or average
r_sum = sum(img[x][y][0] for x in range(w) for y in range(h)) / (w*h)
g_sum = sum(img[x][y][1] for x in range(w) for y in range(h)) / (w*h)
b_sum = sum(img[x][y][2] for x in range(w) for y in range(h)) / (w*h)
print(f"Average: R={r_sum:.1f}, G={g_sum:.1f}, B={b_sum:.1f}")
