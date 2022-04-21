#!/usr/bin/env python
import time
import rclpy
import numpy as np
from networked_sensor.networked_sensor import Sensor
from environ_msgs.msg import TestMsg, NMEAMWV, Pth

class FakeSensor(Sensor):
    def __init__(self):
        super().__init__('fake_sensor')
        self._pub1 = self.create_publisher(TestMsg, 'fake_sensor', 10)
        self._pub2 = self.create_publisher(NMEAMWV, 'fake_wind', 10)
        self._pub3 = self.create_publisher(Pth, 'fake_temp', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.sensor_callback)
        self.i = 0

    def sensor_callback(self):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        msg1 = TestMsg()
        msg1.header.stamp.sec = int(time.time())
        msg1.header.stamp.nanosec = int((time.time()-int(time.time()))*1e9)
        msg1.data = np.random.random()*128
        self._pub1.publish(msg1)

        msg2 = NMEAMWV()
        msg2.header.stamp.sec = int(time.time())
        msg2.header.stamp.nanosec = int((time.time()-int(time.time()))*1e9)
        msg2.wind_speed = np.random.random()*100 # FAST WIND
        msg2.wind_angle = np.random.random()*360 # SPINNY WIND
        msg2.wind_speed_units = "K".encode('ascii')[0]
        self._pub2.publish(msg2)
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
