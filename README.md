# UAVMesh
## Author: Josh Ashley

This is a collection of ROS2 nodes designed to aggregated sensor information from the UAV and send it over a Digimesh network to a central node for realtime analysis. 

This implementation is being created to support the projects under the following [NSF Grant](https://www.nsf.gov/awardsearch/showAward?AWD_ID=1932105).

## Setup

### Parts List

* C94-M8P GPS
* UKY PTH Probe
* Digi XBee Pro 3
* Raspberry Pi 4b

### Connections

* All devices connect to the USB ports on the Pi
* The C94-M8P gpio pin 6 (PPS) connects to Pi GPIO26

### Initial Pi setup

* Install ROS2
* Install Python Dependencies
	* TODO
* Clone repository
* Copy device rules for devices
```bash
cp devices/*.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

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
