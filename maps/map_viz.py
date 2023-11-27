import pickle
import matplotlib.pyplot as plt
import numpy as np

def vertical_split_map_gen(map_dim, num_splits=2):
    # generate map with vertical split
    map = np.zeros((map_dim, map_dim))
    split_width = map_dim // num_splits
    for i in range(num_splits):
        map[:, i*split_width] = 1
    return map

with open('./map_data.pkl', 'rb') as f:
    map = np.array(pickle.load(f))

# reshape map from (1351032,)
map = map.reshape(1373, 984)
new_map = vertical_split_map_gen(500, 5)

# plot cmap with matplotlib
plt.imshow(map, cmap='gray_r')
plt.imshow(new_map, cmap='gray_r')
plt.show()
