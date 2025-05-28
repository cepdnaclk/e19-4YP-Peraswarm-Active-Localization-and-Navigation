import numpy as np
import math
from math import inf
# Parameters
GRID_SIZE = 60
grid_map = np.full((GRID_SIZE, GRID_SIZE), -1)

# LiDAR settings from TurtleBot3
angle_min = 0.0
angle_increment = 0.017501922324299812  # from your data
range_max = 3.5
range_min = 0.12

# Robot position in the grid (center)
robot_x = GRID_SIZE // 2
robot_y = GRID_SIZE // 2

# Sample ranges (use real data here)
ranges =[inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.2728424072265625, 2.2579519748687744, 2.2159152030944824, 2.205627202987671, 2.1818149089813232, 2.1509439945220947, 2.134824514389038, 2.1020302772521973, 2.100283622741699, 2.087991237640381, 2.060208797454834, 2.044337749481201, 2.029569149017334, 2.002847671508789, 2.0081627368927, 1.9978013038635254, 1.9820348024368286, 1.9550225734710693, 1.9431021213531494, 1.9607088565826416, 1.9274044036865234, 1.9416513442993164, 1.9260650873184204, 1.913946270942688, 1.930412769317627, 1.9517232179641724, 2.188430070877075, 2.5079784393310547, inf, inf, inf, inf, inf, inf, inf]   # Fill this with your actual 'ranges' list from the LiDAR message

def polar_to_cartesian(r, angle_rad):
    x = r * math.cos(angle_rad)
    y = r * math.sin(angle_rad)
    return x, y

def world_to_grid(x, y):
    # Convert world coordinates (meters) to grid (assuming 1 cell = 0.1m)
    scale = 10  # 10 cells per meter
    gx = int(robot_x + x * scale)
    gy = int(robot_y + y * scale)
    return gx, gy

def bresenham_line(x0, y0, x1, y1):
    """Yields points on a line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points

# Process each LiDAR reading
for i, r in enumerate(ranges):
    if r == inf or r < range_min or r > range_max:
        continue

    angle = angle_min + i * angle_increment
    obs_x, obs_y = polar_to_cartesian(r, angle)
    grid_x, grid_y = world_to_grid(obs_x, obs_y)

    if 0 <= grid_x < GRID_SIZE and 0 <= grid_y < GRID_SIZE:
        # Mark free space along the ray
        line_points = bresenham_line(robot_x, robot_y, grid_x, grid_y)
        for px, py in line_points[:-1]:  # Exclude obstacle point
            if 0 <= px < GRID_SIZE and 0 <= py < GRID_SIZE:
                if grid_map[py, px] == -1:
                    grid_map[py, px] = 0
        # Mark obstacle
        grid_map[grid_y, grid_x] = 1

# Print map
for row in grid_map:
    print(' '.join(f"{cell:2}" for cell in row))
