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

* XBee radio interface node
```
ros2 run xbee_radio radio 
```

* Transmit over XBee
```
ros2 topic pub transmit std_msgs/String 'data: "Hello World"'
```

* Listen over XBee 
```
ros2 topic echo received
```
