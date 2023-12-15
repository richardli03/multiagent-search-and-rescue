# multiagent-search-and-rescue
The project is a centralized multiagent system designed to efficiently search a space to find a target.

## Table of Contents
- [multiagent-search-and-rescue](#multiagent-search-and-rescue)
  - [Table of Contents](#table-of-contents)
  - [Operation + Configuration](#operation--configuration)
  - [Dependencies:](#dependencies)
  - [Results:](#results)
  - [Extensions:](#extensions)


## Operation + Configuration
To launch the map server that generates our occupancy field, to run it: `ros2 launch search_and_rescue launch_map_server.py map_yaml:=search_and_rescue/maps/new_map.yaml` -- Change the `map_yaml` path as necessary, since the path is relative. 

Then, to run our system, run `ros2 launch search_and_rescue start_multi.py`. To validate that it works, you can run `rqt` or `rviz` and verify that there are multiple Neeto connections. **PLEASE NOTE**: you will have to change the `ip` address that's encoded in the `launch/start_multi.py` file. 


## Dependencies:
- ros2
- numpy
- scikit-learn
  
## Results:
The system works, and results can be found on our website [here](https://sites.google.com/view/comprobo23-search-and-rescue/home)

## Extensions:
While this project is operational, our team had a lot of additional goals we ran out of time to pursue. Here are some things we also wanted to pursue: 

1. A full demo from top to bottom. Our system is technically capable of taking a space that is mapped with LIDAR and splitting it apart, so it'd be cool to see that work from top-to-bottom. Unfortunately, we had to scale down the map that we had already scanned because we didn't have a space that was 9 meters wide. 
2. A more complex space-division system. Our current system just tries to split the space in "half" by dividing the `x` limit by 2. There are a few glaring issues with this, namely that it will rarely divide a non-rectangular space evenly. 
3. A system that automatically moves the Neatos to their correct starting position before searching. Right now the Neatos need to be manually moved to their starting location. 
4. A defined behavior once the object is found. Something we tossed around initially was having all of the Neatos path to the object of interest once it was found, but we didn't get the chance to implement it. 
5. A graph-theory based search system. We have started implementing a graph-theory-based search algorithm. However, we need to integrate it with the neatos infrastructure and include additional features like dynamic goal positions and independent searching.

