import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from xbee_interfaces.msg import Packet

class MsgTransmitter(Node):
    def __init__(self):
        super().__init__('msg_transmitter')
        
        self.declare_parameter('pth_top', 'pth_msg')
        self.declare_parameter('gps_top', 'gps_fix')
        # XBee node MAC address corresponding to the node on the ground station
        self.declare_parameter('gcu_addr', '13A20041D17945')

        pth_top = self.get_parameter('pth_top').value
        gps_top = self.get_parameter('gps_top').value
        self.gcu_addr = self.get_parameter('gcu_addr').value

        self._gps_sub = self.create_subscription(
                NavSatFix,
                gps_top,
                self.gps_callback,
                1)
        self._pth_sub = self.create_subscription(
                String, 
                pth_top, 
                self.pth_callback,
                1)
        self.tx_pub = self.create_publisher(Packet, 'transmit', 10)

    def gps_callback(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000
        utc = datetime.datetime.fromtimestamp(ts)
        utc_str = utc.strftime('%Y%m%d%H%M%S%f')
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        tx_msg = Packet()
        tx_msg.data = f"{utc_str}/{lat},{lon},{alt}"
        tx_msg.dev_addr = self.gcu_addr
        tx_msg.is_broadcast = False
        self.tx_pub.publish(tx_msg)

    def pth_callback(self, msg):
        tx_msg = Packet()
        tx_msg.data = msg.data
        tx_msg.dev_addr = self.gcu_addr
        tx_msg.is_broadcast = False
        self.tx_pub.publish(tx_msg)

def main(args=None):
    rclpy.init(args=args)
    tx_pub = MsgTransmitter()
    rclpy.spin(tx_pub)
    tx_pub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
