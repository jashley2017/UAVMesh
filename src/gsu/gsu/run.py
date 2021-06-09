import rclpy
import datetime
from rclpy.node import Node

from xbee_interfaces.msg import Packet
from sensor_msgs.msg import TimeReference

class GroundStation(Node):
    def __init__(self):
        super().__init__('groundstation')
        self._subscription = self.create_subscription(
                Packet,
                'received',
                self.rx_callback,
                10)
        self.declare_parameter('outfile', '~/outfile.log')

        self.declare_parameter('time_topic', 'gps_time')

        self._subscription = self.create_subscription(
           TimeReference, 
           self.get_parameter('time_topic').value, 
           self.timestamp_creator,
        1)
        self.rel_ts = None
        self.gps_ts = None

    def timestamp_creator(self, time_msg):
        # the time_ref holds the system time 
        self.rel_ts = time_msg.time_ref.sec + time_msg.time_ref.nanosec / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts = time_msg.header.stamp.sec + time_msg.header.stamp.nanosec / 1000000000

    def rx_callback(self, msg):
        if self.rel_ts:
            dt = time.time() - self.rel_ts
            ts = self.gps_ts + dt
            adj_utc = datetime.datetime.fromtimestamp(ts)
        outfile = self.get_parameter('outfile').get_parameter_value().string_value
        self.get_logger().info(f"Writing to: {outfile}")
        with open(outfile, "w") as out: 
            out.write(f"({adj_utc.strftime('%Y:%m:%d:%H:%M:%S:%f')}, {msg.dev_addr}): {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    gs = GroundStation()
    rclpy.spin(gs)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
