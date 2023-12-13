import numpy as np
import heapq

def multi_agent_a_star(graph, agents_start, agents_goal):
    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def a_star_modified(graph, start, goal, visited):
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for next_node in graph.neighbors(current):
                if next_node in visited:
                    continue  # Skip visited nodes

                new_cost = cost_so_far[current] + 1  # Assuming uniform cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current

        # Reconstruct path
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

    visited = set()
    paths = {}
    for agent_id, (start, goal) in enumerate(zip(agents_start, agents_goal)):
        path = a_star_modified(graph, start, goal, visited)
        for node in path:
            visited.add(node)
        paths[agent_id] = path

    return paths
