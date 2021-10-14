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

1. If you are a cloned device, run the initial setup script to ensure no network confusion
```bash 
sudo ./devices/first_setup.sh {device_name}
```
2. Install ROS2
3. Install Python Dependencies
	- TODO
	- pyubx2
	- XBee
	- RPi.GPIO
	- influxdb (gsu)
4. Clone repository
4. a. Update submodules (WARNING: only do so as a part of the FMU setup)
5. Build ros2
```
colcon build
# useful variant for excluding PX4: colcon build --packages-skip-regex px4_.*
source install/setup.bash
```
6. Copy device rules for device aliases
```bash
sudo cp ./devices/*.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```
7. Copy and enable the roslaunch service
	- NOTE: make sure to change the paths to correctly represent where the install folder is.
	- NOTE: uncomment the correct launch file for plane or gsu respectively
```bash
sudo cp ./devices/roslaunch.service /etc/systemd/system/roslaunch.service
sudo systemctl daemon-reload
sudo systemctl enable roslaunch.service
```

## Current Functionality

* Launch plane sensors
```
ros2 launch plane main.launch.py
```

* Launch GSU logger or influxdb interface
```
ros2 launch gsu logger.launch.py
ros2 launch gsu tsdb.launch.py
```

### InfluxDB setup and rationale
InfluxDB is a method to aggregate data on locally in a scalable and queryable way. This will permit usage of all available data in real time. 
A quick way to visualize time-series data coming from InfluxDB is through a web dashboard called Grafana.

* [Install InfluxDB](https://pimylifeup.com/raspberry-pi-influxdb/)
* [Install Grafana](https://grafana.com/tutorials/install-grafana-on-raspberry-pi/)
* [Using InfluxDB with python](https://influxdb-python.readthedocs.io/en/latest/)
