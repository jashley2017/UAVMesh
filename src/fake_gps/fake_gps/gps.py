import time
import rclpy
from networked_sensor.networked_sensor import Sensor
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs.msg import TimeReference

class Gps(Sensor):
    def __init__(self):
        super().__init__('gps')
        self.declare_parameter('time_top', 'gps_time')
        time_top = self.get_parameter('time_top').value
        self._pub = self.create_publisher(TimeReference, time_top)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        '''spoof a time reference'''
        msg = TimeReference()
        msg_hdr = Header()
        system_time = self.get_clock().now().to_msg()
        msg.time_ref = system_time
        msg_hdr.stamp = system_time
        msg.header = msg_hdr
        msg.source = "FAKE"
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gps_pub = Gps()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
