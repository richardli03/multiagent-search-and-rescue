import matplotlib.pyplot as plt
import cv2
import numpy as np
import yaml

def vertical_split_map_gen(map_dim, num_splits=2):
    # generate map with vertical split
    map = np.full(map_dim, 255)
    split_width = map_dim[1] // num_splits
    for i in range(num_splits):
        map[:, i*split_width] = 0
    return map

def yaml_dict(map_name):
    # generate yaml dictionary
    yaml_dict = {
        'image': map_name,
        'mode': 'trinary',
        'resolution': 0.05,
        'origin': [-12.5, -25.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }
    return yaml_dict

new_map = vertical_split_map_gen((500,1000), 5)

map_file_name = 'new_map.pgm'

cv2.imwrite(map_file_name, new_map)

with open(map_file_name.split(".")[0] + ".yaml", 'w') as outfile:
    yaml.dump(yaml_dict(map_file_name), outfile, default_flow_style=None)

newer_map = cv2.imread(map_file_name, cv2.IMREAD_GRAYSCALE)
cv2.imshow('newer_map', newer_map)
cv2.waitKey(0)
