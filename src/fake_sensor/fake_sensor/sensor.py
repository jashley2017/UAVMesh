#!/usr/bin/env python
import time
import rclpy
from rclpy.node import Node
from networked_sensor.networked_sensor import Sensor
from environ_msgs.msg import TestMsg

class FakeSensor(Sensor):
    def __init__(self):
        super().__init__('fake_sensor')
        self.publisher_ = self.create_publisher(TestMsg, 'fake_sensor', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.pth_callback)
        self.i = 0

    def pth_callback(self):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        msg = TestMsg()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((time.time()-int(time.time()))*1e9)
        msg.data = f"{self.i} dummy publish from pth"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing fake data: {self.i}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    pub = FakeSensor()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
