import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# 1. Simulate 2D LiDAR Sensor Data
# -----------------------------
num_points = 360  # 1-degree resolution
angle_min = 0.0
angle_max = 2 * np.pi
angle_increment = (angle_max - angle_min) / num_points
angles = np.linspace(angle_min, angle_max, num_points, endpoint=False)

# Simulated ranges
ranges = np.full(num_points, float('inf'))

# Add a circular wall at 3 meters (45° to 135°)
for i in range(45, 135):
    ranges[i] = 3.0 + np.random.normal(0, 0.05)

# Add a box-shaped obstacle at ~2 meters (210° to 240°)
for i in range(210, 240):
    ranges[i] = 2.0 + np.random.normal(0, 0.05)

# Add a pole at 315° at 1.2 meters
ranges[315] = 1.2

# -----------------------------
# 2. Grid Map Settings
# -----------------------------
meters_per_cell = 0.6  # 0.6 meters per grid cell
grid_size = 10         # 10x10 grid (6m x 6m area)
origin = grid_size // 2  # Robot is at (5,5)
slam_map = np.full((grid_size, grid_size), -1, dtype=int)  # Unknown

# -----------------------------
# 3. Helper: Bresenham Line Algorithm
# -----------------------------
def bresenham(x0, y0, x1, y1):
    """Bresenham's line algorithm for ray tracing."""
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

# -----------------------------
# 4. Process LiDAR Data to Create Grid
# -----------------------------
for r, angle in zip(ranges, angles):
    if not np.isfinite(r):
        continue

    # Cartesian (in meters)
    x = r * np.cos(angle)
    y = r * np.sin(angle)

    # Convert to grid coordinates
    grid_x = int(np.floor(x / meters_per_cell)) + origin
    grid_y = int(np.floor(y / meters_per_cell)) + origin

    # Skip out-of-bounds
    if not (0 <= grid_x < grid_size and 0 <= grid_y < grid_size):
        continue

    # Ray trace from origin to the detected point
    ray_cells = bresenham(origin, origin, grid_x, grid_y)
    for cell in ray_cells[:-1]:  # Free space along the beam
        cx, cy = cell
        if 0 <= cx < grid_size and 0 <= cy < grid_size:
            slam_map[cy, cx] = 0  # Mark as free

    # Last cell is occupied
    slam_map[grid_y, grid_x] = 1

# -----------------------------
# 5. Print Occupancy Grid
# -----------------------------
print("Generated 10x10 SLAM-style Occupancy Grid:")
print(slam_map)


# -----------------------------
# 6. Visualize the Results
# -----------------------------
plt.figure(figsize=(8, 8))

# Visualize Lidar Points
x_points = [r * np.cos(angle) if np.isfinite(r) else np.nan for r, angle in zip(ranges, angles)]
y_points = [r * np.sin(angle) if np.isfinite(r) else np.nan for r, angle in zip(ranges, angles)]
plt.subplot(1, 2, 1)
plt.plot(x_points, y_points, 'bo', markersize=2)
plt.title("Simulated 2D LiDAR Data")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.axis("equal")
plt.grid(True)

# Visualize Grid Map
plt.subplot(1, 2, 2)
plt.imshow(slam_map, cmap='gray', origin='lower')
plt.title("Generated 10x10 Occupancy Grid")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")
cbar = plt.colorbar()
cbar.set_ticks([-1, 0, 1])
cbar.set_ticklabels(['Unknown', 'Free', 'Occupied'])
plt.grid(True)

plt.tight_layout()
plt.show()

