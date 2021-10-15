# Setting up the DJI Telemetry SDK
DJI provides ROS1 nodes for their flight controllers. Therefore, in order to setup the flight controllers on the ROS2 workspace we need to use a bridge. 

## Folder hierarchy 
The following folder structure is assumed.
```
cd ~
git clone git@github.com:jashley2017/UAVMesh.git
git clone https://github.com/dji-sdk/Onboard-SDK-ROS.git
```

## Prerequisites
Install ros1 (melodic or higher) and ros2 (foxy or higher). 

## Building ros1_bridge
The full guide can be found [here]('https://github.com/ros2/ros1_bridge').

```
source /opt/ros/melodic/setup.bash
cd ~/Onboard-SDK-ROS
catkin_make
source ~/.bashrc # clean ROS1 environment
cd ~/UAVMesh
colcon build --symlink-install --packages-skip ros1_bridge
source /opt/ros/melodic/setup.bash # source ros1
source /opt/ros/foxy/local_setup.bash # local source ros2
source ~/Onboard-SDK-ROS/devel/local_setup.bash # local source ros1 ws
source ~/UAVMesh/install/local_setup.bash # local source ros2 ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

## Running DJI on ROS bridge

### Shell 1
```
cd ~/Onboard-SDK-ROS
source devel/setup.bash
roscore
```

### Shell 2 
```
cd ~/Onboard-SDK-ROS
source devel/setup.bash
cd ~/UAVMesh
source install/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

### Shell 3 
```
cd ~/Onboard-SDK-ROS
source devel/setup.bash
roslaunch dji_osdk_ros dji_vehicle_node.launch && rosrun dji_osdk_ros flight_control_node
```

### Shell 4
```
cd ~/UAVMesh
source install/setup.bash
ros2 launch plane main.launch.py
```

# TODO
* This has not yet been tested due to lack of access to DJI FCU.
* Building ROS1 bridge can be simplified by changing the plane package dependencies to require DJI messages.
