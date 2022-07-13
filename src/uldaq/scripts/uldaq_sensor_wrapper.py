#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from networked_sensor.networked_sensor import Sensor
from uldaq_msgs.msg import Buffer, Measurement

class UldaqWrapper(Sensor):
    '''this node's only purpose is to serve as the specification maker for the Uldaq topics and messages'''
    def __init__(self):
        super().__init__('uldaq_wrapper')
        # this is literally all we need to do. the 'Sensor' class with 
        # then format the specification for each of these messages for us. 
        # This cpp -> py conversion is not ideal, maybe write a cpp version of sensor?
        self.buffer_pub = self.create_publisher(Buffer, 'uldaq_buffer', 10)
        self.measure_pub = self.create_publisher(Measurement, 'uldaq_measurement', 10)

def main(args=None):
    rclpy.init(args=args)
    pub = UldaqWrapper()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
