# UAVMesh
## Author: Josh Ashley

This is a collection of ROS2 nodes designed to aggregated sensor information from the UAV and send it over a Digimesh network to a central node for realtime analysis. 

This implementation is being created to support the projects under the following [NSF Grant](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1932105).

## Current Functionality

* Build ros2
```
colcon build
source install/setup.bash
```

* Launch plane sensors
```
ros2 launch plane main.launch.py
```

* Listen on base radio for plane sensor messages
