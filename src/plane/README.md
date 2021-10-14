# plane ros2 node
This node is meant to be a manager of the sensors onboard a plane during flight. 

## Important Contents
- launch/main.launch.py
    - Launch specification for spinning up all available sensors nodes and the plane manager node
- plane/msg_transmitter.py
    - The 'universal' subscriber that subscribes to all sensor data, aligns its time to a GPS time reference, and compresses the message for XBEE transmission.
## Developer notes
- The launch file is complicated, but meant to allow for easy portability between sensor configurations.
- This is the node where all of the interdependencies are. Its not advised nor useful to try to use this node on its own or in some other workspace without the other nodes here alongside it. 
