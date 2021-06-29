# XBee ROS2 Device Interface

This is a ROS2 driver for XBee devices connected through python. It exposed the XBee messages as configurable ROS2 topics for transmitting and receiving.

## Dependencies

* [xbee_interfaces package](https://github.com/jashley2017/UAVMesh/tree/master/src/xbee_interfaces)
* [digi-xbee](https://pypi.org/project/digi-xbee/)

## Topics 

### Subscribed
* /transmit [(xbee_interfaces/Packet.msg)](https://github.com/jashley2017/UAVMesh/blob/master/src/xbee_interfaces/msg/Packet.msg)

### Published
* /received [(xbee_interfaces/Packet.msg)](https://github.com/jashley2017/UAVMesh/blob/master/src/xbee_interfaces/msg/Packet.msg)
