import heapq
import math
import numpy as np

def heuristic(a, b):
    # Euclidean distance
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def get_neighbors(node, grid):
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dx, dy in directions:
        x2, y2 = node[0] + dx, node[1] + dy
        if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
            if grid[x2][y2] == 0:  # Not obstacle
                neighbors.append((x2, y2))
    return neighbors

def extract_local_map(global_grid, robot_pos, window_size=60):
    half = window_size // 2
    x, y = robot_pos
    x_min = max(x - half, 0)
    x_max = min(x + half, len(global_grid))
    y_min = max(y - half, 0)
    y_max = min(y + half, len(global_grid[0]))

    local_map = [row[y_min:y_max] for row in global_grid[x_min:x_max]]
    origin = (x_min, y_min)
    return local_map, origin

def find_edge_local_goal(local_map, start, global_goal):
    rows, cols = len(local_map), len(local_map[0])
    edges = []

    for i in range(rows):
        for j in [0, cols - 1]:
            if local_map[i][j] == 0:
                edges.append((i, j))

    for j in range(cols):
        for i in [0, rows - 1]:
            if local_map[i][j] == 0:
                edges.append((i, j))

    # Map to global
    best_edge = min(edges, key=lambda p: heuristic((p[0] + start[0], p[1] + start[1]), global_goal))
    return best_edge

def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, cost, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in get_neighbors(current, grid):
            temp_g = g_score[current] + 1
            if neighbor not in g_score or temp_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g
                f = temp_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f, temp_g, neighbor))

    return None

# Main local-global planning cycle
def local_global_planner(global_map, robot_pos, global_goal, window_size=60):
    if robot_pos == global_goal:
        return [robot_pos]

    local_map, origin = extract_local_map(global_map, robot_pos, window_size)
    local_start = (robot_pos[0] - origin[0], robot_pos[1] - origin[1])
    local_goal = (global_goal[0] - origin[0], global_goal[1] - origin[1])

    if 0 <= local_goal[0] < len(local_map) and 0 <= local_goal[1] < len(local_map[0]):
        path = a_star(local_map, local_start, local_goal)
    else:
        edge_local_goal = find_edge_local_goal(local_map, origin, global_goal)
        path = a_star(local_map, local_start, edge_local_goal)
        # Map back to global coordinates
        edge_global_goal = (edge_local_goal[0] + origin[0], edge_local_goal[1] + origin[1])
        return path, edge_global_goal

    if path:
        # Map to global coordinates
        global_path = [(p[0] + origin[0], p[1] + origin[1]) for p in path]
        return global_path, global_goal
    else:
        return None, None

# Example usage
if __name__ == "__main__":
    # 200x200 global map with some obstacles
    global_map = np.zeros((200, 200), dtype=int)
    global_map[100:150, 90] = 1  # vertical wall

    robot_pos = (50, 50)
    goal_pos = (180, 130)

    while robot_pos != goal_pos:
        path, next_local_goal = local_global_planner(global_map, robot_pos, goal_pos)
        if path is None:
            print("No path found.")
            break
        print(f"Planning from {robot_pos} -> {next_local_goal}, steps: {len(path)}")
        # Simulate robot moving to local goal
        robot_pos = next_local_goal

