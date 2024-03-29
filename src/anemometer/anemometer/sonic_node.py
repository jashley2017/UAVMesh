#!/usr/bin/env python

import serial
import time
import sys
import os
import pynmea2
import threading

# ros packages
import rclpy
from rclpy.node import Node
from networked_sensor.networked_sensor import Sensor # TODO: need sensor package
from environ_msgs.msg import NMEAXDR, NMEAMWV

AN1_PORT = '/dev/sonic1'
AN2_PORT = '/dev/sonic2'
AN3_PORT = '/dev/sonic3'
AN4_PORT = '/dev/sonic4'

class Anemometer(Sensor):
    def __init__(self):
        super().__init__('anemometer') # Node('anemometer')
        self.wind_pub = self.create_publisher(NMEAMWV, 'wind_speed', 10)
        self.temp_pub = self.create_publisher(NMEAXDR, 'air_temp', 10)
        # parameters come from launch file, otherwise take default
        self.declare_parameter('sonic_port', '/dev/sonic1')
        self.declare_parameter('sonic_baud', 4800)
        an1 = serial.Serial(self.get_parameter('sonic_port').value, self.get_parameter('sonic_baud').value)
        self.read_thread = threading.Thread(target=self.log_nmea, args=(an1,))
        self.running = True
        self.read_thread.start()

    def __del__(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join()

    @staticmethod
    def str_to_roschar(string): # TODO, should be imported
        '''
        roschars are uint8's, this is mean to take a single string 
        symbol (i.e. 'K') and turn in into its correct int
        '''
        return string.encode('ascii')[0]

    def log_nmea(self, usb_dev):
        sample_time = 0 # resolve scope issue
        while rclpy.ok() and self.running:
            try:
                nmea_raw = usb_dev.readline().decode("ascii")
                sample_time = time.time() # nearest millisecond to sample
            except UnicodeDecodeError as e:
                self.get_logger().warning("got decode error, if this continues frequently restart program.")
                continue
            if nmea_raw[0] == '$':
                parsed_nmea = pynmea2.parse(nmea_raw)
                if (parsed_nmea.sentence_type == "XDR"):
                    if(parsed_nmea.id == "TempAir"):
                        ros_msg = NMEAXDR()
                        ros_msg.header.stamp.sec = int(sample_time)
                        ros_msg.header.stamp.nanosec = int((sample_time - int(sample_time))*1000000000)

                        ros_msg.temp = float(parsed_nmea.value)
                        ros_msg.units = self.str_to_roschar(parsed_nmea.units)
                    else:
                        self.get_logger().warning(f"Got unknown NMEA XDR message: {parsed_nmea.id}")
                        continue
                    self.temp_pub.publish(ros_msg)
                elif (parsed_nmea.sentence_type == "MWV"):
                    ros_msg = NMEAMWV()
                    ros_msg.header.stamp.sec = int(sample_time)
                    ros_msg.header.stamp.nanosec = int((sample_time - int(sample_time))*1000000000)

                    ros_msg.wind_speed = float(parsed_nmea.wind_speed)
                    ros_msg.wind_speed_units = self.str_to_roschar(parsed_nmea.wind_speed_units)
                    ros_msg.wind_angle = float(parsed_nmea.wind_angle)
                    self.wind_pub.publish(ros_msg)
                else:
                    self.get_logger().warning(f"Got unknown NMEA message: {parsed_nmea.sentence_type}")
                    continue
            else:
                self.get_logger().warning(f"Got unknown message")
        return None

def main(args=None):
    print("Launching")
    rclpy.init(args=args)
    anem_pub = Anemometer()
    rclpy.spin(anem_pub)
    anem_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
