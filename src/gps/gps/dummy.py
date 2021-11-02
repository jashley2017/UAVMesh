#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Gps(Node):
    def __init__(self):
        super().__init__('gps')
        self.declare_parameter('gps_top', 'gps')
        pub_top = self.get_parameter('gps_top').value
        self.publisher_ = self.create_publisher(String, pub_top, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.gps_callback)
        # on gpio pulse(self.gps_callback)
        self.i = 0

    def gps_callback(self):
        msg = String()
        msg.data = f"{self.i} Dummy publish from gps"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing gps: {self.i}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    gps_pub = Gps()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

	
