import rclpy
from rclpy.node import Node

from xbee_interfaces.msg import Packet

class GroundStation(Node):
    def __init__(self):
        super.__init__('groundstation')
        self._subscription = self.create_subscription(
                Packet,
                'received',
                self.rx_callback,
                10)
        self.declare_parameter('outfile', '~/outfile.log')
    def rx_callback(self, msg):
        outfile = self.get_parameter('outfile').get_parameter_value().string_value
        self.get_logger().info(f"Writing to: {outfile}")
        with open(outfile, "w") as out: 
            out.write(f"({msg.timestamp}, {msg.dev_addr}): {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    gs = GroundStation()
    rclpy.spin(gs)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
