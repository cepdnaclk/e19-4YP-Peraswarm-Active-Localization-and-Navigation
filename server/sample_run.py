import numpy as np
import math
from math import inf
# Map config
MAP_SIZE = 60
MAP_CENTER = MAP_SIZE // 2
RESOLUTION = 0.05  # 1 cell = 5 cm, adjust depending on use case

# LiDAR config (from TurtleBot3 spec)
angle_min = 0.0
angle_increment = 0.0175  # rad (~1 degree)
range_min = 0.12
range_max = 3.5

# Sample LiDAR ranges (you should replace this with actual `ranges` list)
ranges = [ 1, 1, 1, 1, 1, 1, 1, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.2728424072265625, 2.2579519748687744, 2.2159152030944824, 2.205627202987671, 2.1818149089813232, 2.1509439945220947, 2.134824514389038, 2.1020302772521973, 2.100283622741699, 2.087991237640381, 2.060208797454834, 2.044337749481201, 2.029569149017334, 2.002847671508789, 2.0081627368927, 1.9978013038635254, 1.9820348024368286, 1.9550225734710693, 1.9431021213531494, 1.9607088565826416, 1.9274044036865234, 1.9416513442993164, 1.9260650873184204, 1.913946270942688, 1.930412769317627, 1.9517232179641724, 2.188430070877075, 2.5079784393310547, inf, inf, inf, inf, inf, inf, inf]  # <- insert the list of float distances here from your sensor

# Initialize the grid map
grid_map = np.full((MAP_SIZE, MAP_SIZE), -1)

# Process each LiDAR reading
for i, r in enumerate(ranges):
    if np.isinf(r) or np.isnan(r):
        continue
    if r < range_min or r > range_max:
        continue

    angle = angle_min + i * angle_increment

    # Calculate obstacle point in meters
    end_x = r * math.cos(angle)
    end_y = r * math.sin(angle)

    # Convert to map indices
    end_ix = int(end_x / RESOLUTION) + MAP_CENTER
    end_iy = int(end_y / RESOLUTION) + MAP_CENTER

    # Draw ray (free space) using Bresenham-like line interpolation
    steps = int(r / RESOLUTION)
    for step in range(steps):
        x = (step * math.cos(angle)) / RESOLUTION
        y = (step * math.sin(angle)) / RESOLUTION
        ix = int(x) + MAP_CENTER
        iy = int(y) + MAP_CENTER
        if 0 <= ix < MAP_SIZE and 0 <= iy < MAP_SIZE and grid_map[iy, ix] == -1:
            grid_map[iy, ix] = 0  # Free space

    # Mark obstacle
    if 0 <= end_ix < MAP_SIZE and 0 <= end_iy < MAP_SIZE:
        grid_map[end_iy, end_ix] = 1

# Print the map for visualization (optional)
for row in grid_map:
    print(" ".join(str(cell) for cell in row))
