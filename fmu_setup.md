# Flight Controller Companion Computer ROS2 Setup Guide

## Hardware Requirements

1. FMU device
2. Companion Computer Compatible with ROS2
3. FTDI cable
	- Follow connection guide from the [PX4 user guide]('https://docs.px4.io/master/en/companion_computer/pixhawk_companion.html')
4. MicroSD card

## Companion Computer Setup
1. In your ROS2 workspace add submodules for px4_ros_com and px4_msgs to your source. 
2. Install FastRTPS (DDS) following the guide [here]('https://docs.px4.io/v1.12/en/ros/ros2_comm.html#install-fast-dds').
3. Build the workspace and subsequent nodes using colcon build. 
4. Additional information for this setup can be found [here]('https://dev.px4.io/v1.11_noredirect/en/middleware/micrortps.html#px4_ros_com').

## FMU device setup
1. On a separate development computer (not the companion computer), download the PX4-Autopilot source code.
```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```
2. Find the most recent version of PX4-Autopilot that has the rtps firmware for your FMU version.
```sh
git log --all --full-history -- ./boards/{board_type}/fmu-{fmu version}/rtps.cmake
# example
git log --all --full-history -- ./boards/px4/fmu-v3/rtps.cmake
git checkout {most recent git hash}
```
3. Plug the FMU device in over the USB port or whatever I/O used for flashing the device. 
4. Locate the device path to the FMU. (looks something like /dev/ttyACM0)
5. Flash the device with the rtps firmware. 
```sh
make {board_type}_fmu-{fmu version}_rtps upload 
# example 
make px4_fmu-v3_rtps upload 
```
6. Given no errors. The firmware should now be loaded onto the board. For more PX4-Autopilot information, visit [here]('https://docs.px4.io/master/en/dev_setup/building_px4.html')
7. Plug in the FMU device to a computer with QGroundControl to configure RTPS parameters.
8. After the board is loaded. Search the parameters for RTPS and enable RTPS on the desired port.
9. Take note of the [serial port mapping]('https://docs.px4.io/master/en/hardware/serial_port_mapping.html') to your port.
10. Also take note of the baudrate (this should be >=460800) of the selected port and ensure that mavlink is disabled on the port. 
11. Disconnect the device from QGroundControl.
12. On the microSD card create the following file: "etc/extras.txt" and insert the following to that file. 
```
set +e 
micrortps_client start -d {desired port} -b {baudrate}
set -e
```
13. Eject the microSD and plug it into the FMU.

## Running the Companion Computer Setup
1. Connect the FMU and Companion Computer via the FTDI cable into the port we set RTPS enabled and power the FMU.
2. Note the new USB device that should look like: "/dev/ttyACMX" or "/dev/ttyUSBX".
3. Open 2 terminals on your companion computer and do the following. 
```sh 
# both
cd {ros workspace}
source install/setup.bash
# terminal 1
micrortps_agent -d {usb device} -b {baudrate}
# terminal 2
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
4. If everything worked well, it will now prompt you with received sensor combined data. 

## Developers Notes
* The majority of this guide was derived from [PX4 User Guide]('https://docs.px4.io/') and experimentation.
* I would not recommend attempting this without the following skills:
	- Interpreting make/compiler errors.
	- Understanding of a live Linux Environment.
	- Decent understanding of ROS2 and DDS'es
	- Patience and googling ability.
* Configuration of the uorb-to-rtps messages are done through the following files (requires rebuild):
	- FMU: PX4-Autopilot/msg/tools/uorb_rtps_messages_ids.yaml
	- Companion: px4_ros_com/templates/uorb_rtps_message_ids.yaml
	- How these config files work can be found on the [micrortps page]('https://docs.px4.io/master/en/middleware/micrortps.html#supported-uorb-messages')
* Rolling back the firmware is somewhat of a hack because of the dropped support of legacy devices for RTPS firmware. 
It is not advised, but with hardware older than FMUv5 it is currently the only way. 
* This guide is underdeveloped, please contact the maintainer if there is any missing information in it. Additionally 
you can consult the PX4 User Guide for details. 
