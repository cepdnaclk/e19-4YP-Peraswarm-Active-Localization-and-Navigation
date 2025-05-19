import numpy as np
import matplotlib.pyplot as plt

# LiDAR scan data
ranges = [float('inf')] * 100 + [
    2.786, 2.756, 2.742, 2.734, 2.706,
    2.698, 2.661, 2.662, 2.645, 2.633,
    2.623, 2.620, 2.575, 2.596, 2.578,
    2.570, 2.574, 2.804, 3.318
] + [float('inf')] * 100

# LiDAR parameters
angle_min = 0.0
angle_max = 2 * np.pi
angle_increment = (angle_max - angle_min) / len(ranges)
angles = np.arange(angle_min, angle_max, angle_increment)

# Map configuration
meters_per_cell = 0.6
grid_size = 10
origin = grid_size // 2  # Robot is at (5,5)
slam_map = np.full((grid_size, grid_size), -1, dtype=int)  # Unknown

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

# Process LiDAR data
for r, angle in zip(ranges, angles):
    if not np.isfinite(r):
        continue

    # Cartesian (in meters)
    x = r * np.cos(angle)
    y = r * np.sin(angle)

    # Convert to grid indices
    grid_x = int(np.floor(x / meters_per_cell)) + origin
    grid_y = int(np.floor(y / meters_per_cell)) + origin

    # Skip if out of bounds
    if not (0 <= grid_x < grid_size and 0 <= grid_y < grid_size):
        continue

    # Ray tracing from origin to hit cell
    ray_cells = bresenham(origin, origin, grid_x, grid_y)
    for cell in ray_cells[:-1]:  # All but the last are free
        cx, cy = cell
        if 0 <= cx < grid_size and 0 <= cy < grid_size:
            slam_map[cy, cx] = 0  # Free

    # Last cell is occupied
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        slam_map[grid_y, grid_x] = 1

# Visualize the occupancy map
##plt.imshow(slam_map, cmap='gray', origin='lower')
##plt.title("SLAM-style 10x10 Occupancy Grid")
##plt.xlabel("X Cells")
##plt.ylabel("Y Cells")
##cbar = plt.colorbar()
##cbar.set_ticks([-1, 0, 1])
##cbar.set_ticklabels(['Unknown', 'Free', 'Occupied'])
##plt.grid(True)
##plt.show()

# Optional: print 2D array
print("Occupancy Grid Map (SLAM-style):")
print(slam_map)
