# planner_core.py


import math
import heapq

# -----------------------------
# Occupancy grid map
# -----------------------------
class OccupancyGrid:
    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.grid = [[0 for _ in range(width)] for _ in range(height)]

    def world_to_grid(self, x, y):
        j = int((x - self.origin_x) / self.resolution)
        i = int((y - self.origin_y) / self.resolution)
        return i, j

    def grid_to_world(self, i, j):
        x = self.origin_x + (j + 0.5) * self.resolution
        y = self.origin_y + (i + 0.5) * self.resolution
        return x, y

    def in_bounds(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def is_free(self, i, j):
        return self.in_bounds(i, j) and self.grid[i][j] == 0

    def add_rect_obstacle(self, x_min, y_min, x_max, y_max):
        i0, j0 = self.world_to_grid(x_min, y_min)
        i1, j1 = self.world_to_grid(x_max, y_max)

        for i in range(min(i0, i1), max(i0, i1) + 1):
            for j in range(min(j0, j1), max(j0, j1) + 1):
                if self.in_bounds(i, j):
                    self.grid[i][j] = 1

    def inflate_obstacles(self, inflation_radius_m):
        radius_cells = int(math.ceil(inflation_radius_m / self.resolution))
        inflated = [row[:] for row in self.grid]

        for i in range(self.height):
            for j in range(self.width):
                if self.grid[i][j] == 1:
                    for di in range(-radius_cells, radius_cells + 1):
                        for dj in range(-radius_cells, radius_cells + 1):
                            ni, nj = i + di, j + dj
                            if self.in_bounds(ni, nj):
                                inflated[ni][nj] = 1

        self.grid = inflated

    def neighbors(self, node):
        i, j = node
        result = []

        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                if di == 0 and dj == 0:
                    continue

                ni, nj = i + di, j + dj
                if self.is_free(ni, nj):
                    result.append((ni, nj))

        return result

    def line_of_sight(self, a, b):
        i0, j0 = a
        i1, j1 = b

        di = abs(i1 - i0)
        dj = abs(j1 - j0)

        si = 1 if i1 > i0 else -1
        sj = 1 if j1 > j0 else -1

        err = dj - di
        i, j = i0, j0

        while True:
            if not self.is_free(i, j):
                return False

            if i == i1 and j == j1:
                return True

            e2 = 2 * err

            if e2 > -di:
                err -= di
                j += sj

            if e2 < dj:
                err += dj
                i += si


def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def theta_star(grid_map, start, goal):
    open_heap = []
    heapq.heappush(open_heap, (0.0, start))

    g = {start: 0.0}
    parent = {start: start}
    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        if current == goal:
            break

        closed.add(current)

        for neighbor in grid_map.neighbors(current):
            if neighbor in closed:
                continue

            if grid_map.line_of_sight(parent[current], neighbor):
                candidate_parent = parent[current]
                candidate_g = g[candidate_parent] + euclidean(candidate_parent, neighbor)
            else:
                candidate_parent = current
                candidate_g = g[current] + euclidean(current, neighbor)

            if candidate_g < g.get(neighbor, float("inf")):
                g[neighbor] = candidate_g
                parent[neighbor] = candidate_parent
                f = candidate_g + euclidean(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    if goal not in parent:
        return []

    path = [goal]
    node = goal

    while node != start:
        node = parent[node]
        path.append(node)

    path.reverse()
    return path