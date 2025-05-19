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
meters_per_cell = 0.6  # Each cell = 0.6 meters
grid_size = 10         # 10x10 grid
max_range_meters = meters_per_cell * grid_size  # 6 meters

# Initialize empty map
grid_map = np.zeros((grid_size, grid_size), dtype=int)

# Sensor position is at center of the map
origin_x = origin_y = grid_size // 2

# Process LiDAR data
for r, angle in zip(ranges, angles):
    if not np.isfinite(r):
        continue

    # Convert polar to Cartesian
    x = r * np.cos(angle)
    y = r * np.sin(angle)

    # Convert meters to grid coordinates
    grid_x = int(np.floor(x / meters_per_cell)) + origin_x
    grid_y = int(np.floor(y / meters_per_cell)) + origin_y

    # Check bounds
    if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
        grid_map[grid_y, grid_x] = 1  # Mark as occupied

# Display the map
plt.imshow(grid_map, cmap='Greys', origin='lower')
plt.title("10x10 Occupancy Grid Map from LiDAR")
plt.xlabel("X Cells")
plt.ylabel("Y Cells")
plt.grid(True)
plt.colorbar(label='Occupied')
plt.show()
