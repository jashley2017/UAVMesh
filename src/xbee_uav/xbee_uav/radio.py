import rclpy
from rclpy.node import Node

# TODO: add this to rosdep or a venv
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
from digi.xbee.models.address import XBee64BitAddress

from std_msgs.msg import String

# XBee node MAC address corresponding to the node on the ground station
GCU_NODE_ADDR="13A20041D17945"

class XBeeRadio(Node): 
    def __init__(self):
        super().__init__('xbee_radio')
        
        # setup XBee radio and add callback to publish
        self.device = XBeeDevice("/dev/ttyUSB0", 9600)
        self.device.open()
        self.device.add_data_received_callback(self.rx_callback)

        self._publisher = self.create_publisher(String, 'received', 10)
        self._subscription = self.create_subscription(
                String, 
                'transmit', 
                self.tx_callback, 
                10)
        self._subscription
    
    def tx_callback(self, msg):
        ''' callback for data wanting to be transmitted by the XBee radio '''
        self.get_logger().info(f"Transmitting: {msg.data}")
        remote_device = RemoteXBeeDevice(self.device, XBee64BitAddress.from_hex_string(GCU_NODE_ADDR))
        self.device.send_data(remote_device, msg.data)

    def rx_callback(self, msg):
        ''' callback for data received by the XBee radio '''
        data = msg.data.decode("utf8")
        self.get_logger().info(f"Received: {data}")
        ros_msg = String()
        ros_msg.data = data
        self._publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    xbee_radio = XBeeRadio()
    rclpy.spin(xbee_radio)
    xbee_radio.device.close()
    xbee_radio.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
