import rclpy
from rclpy.node import Node

# TODO: add this to rosdep or a venv
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress

from std_msgs.msg import String
from xbee_interfaces.msg import Packet


class XBeeRadio(Node): 
    def __init__(self):
        super().__init__('xbee_radio')

        self.declare_parameter('xbee_port', '/dev/xbee')
        self.declare_parameter('xbee_baud', 9600)
        port = self.get_parameter('xbee_port').value
        baud = self.get_parameter('xbee_baud').value
        
        # setup XBee radio and add callback to publish
        self.device = XBeeDevice(port, baud)
        self.device.open()
        self.device.add_data_received_callback(self.rx_callback)

        self._publisher = self.create_publisher(Packet, 'received', 10)
        self._subscription = self.create_subscription(
                Packet, 
                'transmit', 
                self.tx_callback, 
                10)
        self._subscription
    
    def tx_callback(self, msg):
        ''' callback for data wanting to be transmitted by the XBee radio '''
        self.get_logger().info(f"Transmitting: {msg.data}")
        remote_device = RemoteXBeeDevice(self.device, XBee64BitAddress.from_hex_string(msg.dev_addr))
        self.device.send_data(remote_device, msg.data)

    def rx_callback(self, msg):
        ''' callback for data received by the XBee radio '''
        data = msg.data.decode("utf8")
        dev_addr = str(msg.remote_device.get_64bit_addr())
        self.get_logger().info(f"Received: {data} from {dev_addr} at time: {msg.timestamp}")
        packet = Packet()
        packet.data = data
        packet.dev_addr = dev_addr
        packet.timestamp = msg.timestamp
        packet.is_broadcast = msg.is_broadcast 
        self._publisher.publish(packet)

def main(args=None):
    rclpy.init(args=args)
    xbee_radio = XBeeRadio()
    rclpy.spin(xbee_radio)
    xbee_radio.device.close()
    xbee_radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
