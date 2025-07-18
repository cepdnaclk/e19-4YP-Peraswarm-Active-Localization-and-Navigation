import heapq
import math

def a_star(grid, start, goal):
    def heuristic(a, b):
        # Euclidean distance
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def get_neighbors(node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-connected
        neighbors = []
        for dx, dy in directions:
            x2, y2 = node[0] + dx, node[1] + dy
            if 0 <= x2 < len(grid) and 0 <= y2 < len(grid[0]):
                if grid[x2][y2] == 0:  # Not an obstacle
                    neighbors.append((x2, y2))
        return neighbors

    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + 1  # Cost from current to neighbor
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

    return None  # No path found


# Example usage:
if __name__ == "__main__":
    # 0 = free space, 1 = obstacle
    grid = [
        [0, 0, 0, 0, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ]
    start = (0, 0)
    goal = (3, 0)
    path = a_star(grid, start, goal)

    if path:
        print("Path found:", path)
        print("Path length:", len(path) - 1)
    else:
        print("No path found.")
