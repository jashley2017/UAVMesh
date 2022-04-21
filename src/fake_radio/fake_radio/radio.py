import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg  import String
from xbee_interfaces.msg import Packet

class Radio(Node):
    def __init__(self):
        super().__init__('fake_radio')
        latching_qos = QoSProfile(depth=1, # QOS profile that queues message for subscriber
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self._pub = self.create_publisher(Packet, 'received', latching_qos)
        self._sub = self.create_subscription(Packet, 'transmit', self.callback, latching_qos)
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

def main(args=None):
    rclpy.init(args=args)
    radio_pub = Radio()
    rclpy.spin(radio_pub)
    radio_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
