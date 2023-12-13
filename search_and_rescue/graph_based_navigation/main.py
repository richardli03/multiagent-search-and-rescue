import networkx as nx
import numpy as np
import cv2
from multiastar import multi_agent_a_star

def image_to_graph(image_path):
    arr = cv2.imread(image_path)
    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)

    arr[10:20, 200] = 255

    # cut image to 500x500
    arr = arr[:500, :500]

    graph = nx.Graph()

    rows, cols = arr.shape
    for i in range(rows):
        for j in range(cols):
            if arr[i, j] > 0:  # White pixel, add node
                graph.add_node((i, j))
                if i > 0 and arr[i - 1, j] > 0:  # Connect to upper node
                    graph.add_edge((i, j), (i - 1, j))
                if j > 0 and arr[i, j - 1] > 0:  # Connect to left node
                    graph.add_edge((i, j), (i, j - 1))

    return graph

def a_star_search(graph, start, goal):
    return nx.astar_path(graph, start, goal)

graph = image_to_graph('../maps/new_map.pgm')

# a_star_search
# path = a_star_search(graph, (10, 10), (499, 210))

# multi-agent A*
agents_start = [(10, 10), (10, 14)]
agents_goal = [(499, 210), (499, 220)]

paths = multi_agent_a_star(graph, agents_start, agents_goal)

print(paths)

# visualize the paths
arr = cv2.imread('images/map.pgm')
for agent_id, path in paths.items():
    for node in path:
        if agent_id == 0:
            arr[node] = [255, 0, 0]
        else:
            arr[node] = [0, 0, 255]
cv2.imshow('path', arr)
cv2.waitKey(0)
