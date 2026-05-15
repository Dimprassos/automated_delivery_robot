import heapq
import math


class GraphPlanner:
    def __init__(self, nodes, edges):
        self.nodes = nodes
        self.graph = {name: [] for name in nodes}

        for a, b in edges:
            distance = self.distance_between(a, b)
            self.graph[a].append((b, distance))
            self.graph[b].append((a, distance))

    def distance_between(self, a, b):
        ax, ay = self.nodes[a]
        bx, by = self.nodes[b]

        return math.hypot(bx - ax, by - ay)

    def heuristic(self, node, goal):
        return self.distance_between(node, goal)

    def plan(self, start, goal):
        if start not in self.nodes:
            raise ValueError(f"Unknown start node: {start}")

        if goal not in self.nodes:
            raise ValueError(f"Unknown goal node: {goal}")

        frontier = [(0.0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for neighbor, edge_cost in self.graph[current]:
                new_cost = cost_so_far[current] + edge_cost

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal not in came_from:
            raise ValueError(f"No route from {start} to {goal}")

        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = came_from[current]

        path.reverse()
        return path

    def path_distance(self, path):
        total = 0.0

        for index in range(len(path) - 1):
            total += self.distance_between(path[index], path[index + 1])

        return total
