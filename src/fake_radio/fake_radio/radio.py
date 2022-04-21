import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg  import String
from xbee_interfaces.msg import Packet

class Radio(Node):
    def __init__(self):
        super().__init__('fake_radio')
        self._pub = self.create_publisher(Packet, 'received', 10)
        self._sub = self.create_subscriber(Packet, 'transmit', self.callback, 10)
    def callback(self, msg):
        '''
        needs to catch sensor spec messages (code 0) and spit back out acknowledgements (ack)
        '''
        if msg.data[0] == b'0':
            spec_msg = str(struct.pack(str(len(msg.data)) + 'c', *msg.data), encoding='ascii')
            sensor_code = spec_msg.split(',')[1]
            ack = Packet()
            ack.data = [b'0', struct.pack('B', int(sensor_code))]
            ack.dev_addr = msg.dev_addr
            self._pub.publish(ack)
        # self.get_logger().info('got a packet!')

def main(args=None):
    rclpy.init(args=args)
    radio_pub = Radio()
    rclpy.spin(radio_pub)
    radio_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
