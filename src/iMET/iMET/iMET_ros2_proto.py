import os
import time
import serial
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from environ_msgs.msg import IMET
from networked_sensor.networked_sensor import Sensor # TODO: need sensor package


class iMET(Sensor):
    def __init__(self):
        super().__init__('iMET')
        self.read_thread = False
        self.declare_parameter('iMET_topic', 'iMET_data')
        self.iMET_pub = self.create_publisher(IMET, self.get_parameter('iMET_topic').value, 10)
        self.declare_parameter('iMET_port', '/dev/iMET1')
        self.declare_parameter('iMET_baud', '57600')
        #TODO: double check baud, may need to add parity and stop bits arguments
        iMET1 = serial.Serial(self.get_parameter('iMET_port').value, self.get_parameter('iMET_baud').value, 
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1)
        self.read_thread = threading.Thread(target=self.log_iMET, args=(iMET1,)) #TODO
        self.running = True
        self.read_thread.start()

    def __del__(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join()

    def log_iMET(self, usb_dev):
        sample_time = 0
        while rclpy.ok() and self.running:
            try:
                iMET_raw = usb_dev.readline().decode("ascii")
                parsed = iMET_raw.split(',')
                self.get_logger().info(f"got {iMET_raw}, {len(parsed)}")
                sample_time = datetime.strptime(parsed[4] + ',' + parsed[5], '%Y/%m/%d,%H:%M:%S').timestamp()
                ros_iMET_msg = IMET()
            except (UnicodeDecodeError, IndexError, ValueError) as e:
                self.get_logger().warning(f"Got decode error: {e} If this continues to occur, restart program.")
                continue

            if parsed[0] == 'XQ':
                ros_iMET_msg.pressure = int(parsed[1])
                ros_iMET_msg.temp = int(parsed[2])
                ros_iMET_msg.rel_humidity = int(parsed[3])
                ros_iMET_msg.humidity_temp = 0#int(parsed[4]) TODO: not there ? 
                ros_iMET_msg.longitude = int(parsed[6])
                ros_iMET_msg.latitude = int(parsed[7])
                ros_iMET_msg.altitude = int(parsed[8])
                ros_iMET_msg.num_sat = int(parsed[9])
            else:
                self.get_logger().warning("bad IMET message")
            ros_iMET_msg.header.stamp.sec = int(sample_time)
            ros_iMET_msg.header.stamp.nanosec = int((sample_time - int(sample_time))*1000000000)

            self.iMET_pub.publish(ros_iMET_msg)
            print("something went wrong with logging iMET")
        return None

def main(args=None):
    print("launching")
    rclpy.init(args=args)
    iMET_pub = iMET()
    rclpy.spin(iMET_pub)
    iMET_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

