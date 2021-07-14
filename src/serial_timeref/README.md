# Time-synchronized ROS2 Serial Message interface

This package reads a serial message and associates it with a reference time for which it came in. Given that the USB device is 
receiving data sufficiently fast this can provide an accurate GPS time of when the data was recorded.

It is meant to be generic and parameter configurable with the parameters described below in your ROS2 environment. 

This is not utilizing microROS2 and is simply regex parsing the incoming serial strings. Only time synchronization code would 
be needed given the microcontroller was publishing messages over microROS2. 

## Topics

### Subscribes
* parameter('time_topic') default: /gps_time [(sensor_msgs/TimeReference.msg)](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/TimeReference.html)

### Publishes
* parameter('sensor_top') default: /serial1 (parameter('sensor_msg') default: [std_msgs/String.msg](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))
    - NOTE: Using importlib, the code can receive your message type as a parameter. The pythonic syntax for modules should be followed (i.e. "sensor_msgs.msg.TimeReference").

## Parameters

| Name             | Type                     | Default               | Description                                                                                                                            |
|------------------|--------------------------|-----------------------|----------------------------------------------------------------------------------------------------------------------------------------|
| sensor_top       | String                   | 'serial1'             | Topic to publish serial message.                                                                                                       |
| sensor_msg       | String                   | 'std_msgs.msg.String' | Serial message type.                                                                                                                   |
| sensor_port      | String                   | '/dev/ttyUSB0'        | Port the serial device is on.                                                                                                          | 
| sensor_baud      | Int                      | 9600                  | Baudrate of serial device.                                                                                                             | 
| sensor_reg       | RegEx                    | None                  | [Keyed RegEx](https://www.geeksforgeeks.org/re-matchobject-groupdict-function-in-python-regex/) to parse the serial strings into serial messages. Key's should be the same name as message attributes.                     | 
| time_topic       | String                   | 'gps_time'            | TimeReference Topic                                                                                                                    | 
| sensor_key_types | Dict of Strings and Ints | None                  | A dictionary to tell the RegEx what type each matched element is. The keys should be the same as in the RegEx and message attributes.  | 

sensor_reg and sensor_key_types *MUST* be set for this to function properly but no reasonable default was known.
