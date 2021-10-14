# gps ros2 node
UBlox GPS node for ROS2.
## Important Files
- gps/gps.py
    - Records the incoming NAV_PVT message at each timepulse and transmits it as a TimeReference and NavSatFix message.
## Developers notes
- Most other files are for GPS configuration and resetting purposes, if the correct messages are coming in from the GPS do not bother with the other scripts. 
- Currently the only Raspberry Pi dependent package. 
- Ensure through whatever configuration that NAV-PVT is transmitting. 
