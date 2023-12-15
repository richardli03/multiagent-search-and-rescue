import numpy as np
import heapq


def heuristic(a, b):
    """
    Calculates the Euclidean distance between two points.

    Parameters:
    a, b (tuple): The points to calculate the distance between.

    Returns:
    float: The Euclidean distance between the points.
    """
    return np.linalg.norm(np.array(a) - np.array(b))


def a_star_modified(graph, start, goal, visited):
    """
    Performs a modified A* search on the graph from start to goal, skipping nodes that have already been visited.

    Parameters:
    graph (networkx.Graph): The graph to search.
    start (tuple): The start node.
    goal (tuple): The goal node.
    visited (set): The set of nodes that have already been visited.

    Returns:
    list: The path from start to goal.
    """
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    while frontier:
        # Get the node with the lowest cost
        _, current = heapq.heappop(frontier)

        if current == goal:
            break

        # loop through neighbors
        for next_node in graph.neighbors(current):
            if next_node in visited:
                continue  # Skip visited nodes
            
            # Calculate cost to reach neighbor
            new_cost = cost_so_far[current] + 1  # Assuming uniform cost
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                # update cost to reach neighbor
                cost_so_far[next_node] = new_cost
                # update priority of neighbor
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


def multi_agent_a_star(graph, agents_start, agents_goal):
    """
    Performs A* search for multiple agents on the graph from their respective start to goal nodes.

    Parameters:
    graph (networkx.Graph): The graph to search.
    agents_start (list): The list of start nodes for each agent.
    agents_goal (list): The list of goal nodes for each agent.

    Returns:
    dict: The paths from start to goal for each agent. The keys are the agent IDs and the values are the paths.
    """
    visited = set()
    paths = {}
    for agent_id, (start, goal) in enumerate(zip(agents_start, agents_goal)):
        path = a_star_modified(graph, start, goal, visited)
        for node in path:
            visited.add(node)
        paths[agent_id] = path

    return paths
