import matplotlib.pyplot as plt
import cv2
import numpy as np

def vertical_split_map_gen(map_dim, num_splits=2):
    # generate map with vertical split
    map = np.full(map_dim, 255)
    split_width = map_dim[1] // num_splits
    for i in range(num_splits):
        map[:, i*split_width] = 0
    return map

new_map = vertical_split_map_gen((500,1000), 5)

cv2.imwrite('new_map.pgm', new_map)

newer_map = cv2.imread('new_map.pgm', cv2.IMREAD_GRAYSCALE)
cv2.imshow('newer_map', newer_map)
cv2.waitKey(0)
