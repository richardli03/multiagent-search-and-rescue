import networkx as nx
import numpy as np
import cv2
from multiastar import multi_agent_a_star
import matplotlib.pyplot as plt


def modify_map(arr):
    """
    Modifies the input array to create openings in the wall.

    Parameters:
    arr (numpy.ndarray): The input array representing the map.

    Returns:
    numpy.ndarray: The modified array.
    """
    # create openings in the wall
    arr[10:20, 200] = 255
    arr[60:70, 200] = 255

    return arr


def colors_to_rgb(clr):
    """
    TODO: Find a better way to do this.
    Converts color abbreviations to their corresponding RGB values.

    Parameters:
    clr (str): The color abbreviation.

    Returns:
    tuple: The corresponding RGB values.
    """
    if clr == 'r':
        return (255, 0, 0)
    elif clr == 'b':
        return (0, 0, 255)
    elif clr == 'g':
        return (0, 255, 0)
    elif clr == 'c':
        return (0, 255, 255)
    elif clr == 'm':
        return (255, 0, 255)
    elif clr == 'y':
        return (255, 255, 0)
    elif clr == 'k':
        return (0, 0, 0)
    elif clr == 'w':
        return (255, 255, 255)
    return (0, 0, 0)


def image_to_graph(image_path):
    """
    Converts an image to a graph where each white pixel is a node. 
    Avoids black pixels (obstacles).

    Parameters:
    image_path (str): The path to the image file.

    Returns:
    networkx.Graph: The resulting graph.
    """
    arr = cv2.imread(image_path)
    arr = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)

    arr = modify_map(arr)

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


MAP_PATH = '../maps/graph_nav.pgm'

graph = image_to_graph(MAP_PATH)

# define the start and goal nodes for each agent
agents_start = [(15, 10), (55, 10), (45, 10)]
agents_goal = [(55, 210), (15, 210), (35, 210)]

paths = multi_agent_a_star(graph, agents_start, agents_goal)

markers = ['o', 'x', 'v', '^', '<', '>', 's', 'p', '*', 'h', 'H', 'D', 'd']
colors = ['r', 'b', 'k', 'c', 'm', 'y', 'g', 'w']

# visualize the paths
arr = cv2.imread(MAP_PATH)
arr = modify_map(arr)
for agent_id, path in paths.items():
    for node in path:
        arr[node] = colors_to_rgb(colors[agent_id])

# plot the image
plt.imshow(arr)

for agent in range(len(agents_start)):
    plt.scatter(agents_start[agent][1], agents_start[agent][0],
                c=colors[agent], marker=markers[agent], label=f'Agent {agent} Start')
    plt.scatter(agents_goal[agent][1], agents_goal[agent][0],
                c=colors[agent], marker=markers[agent], label=f'Agent {agent} Goal')

plt.legend()
plt.show()
