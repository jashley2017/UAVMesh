import struct
import re
import datetime
import rclpy
from rclpy.node import Node
from environ_msgs.msg import Pth
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
                Pth, 
                pth_top, 
                self.pth_callback,
                1)
        self.tx_pub = self.create_publisher(Packet, 'transmit', 10)
        self.gps_code = b'1'
        self.pth_code = b'2'

    def _create_bytelist(self, var, bytelen=4, vartype='f'):
        # TODO: can probably autodetect the bytelen for vartype
        return list(struct.unpack(str(bytelen) + 'c', struct.pack(vartype, var)))

    def gps_callback(self, msg):
        msg_data = [self.gps_code]
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
        msg_data += self._create_bytelist(ts, bytelen=8, vartype='d')
        msg_data += self._create_bytelist(msg.latitude)
        msg_data += self._create_bytelist(msg.longitude)
        msg_data += self._create_bytelist(msg.altitude)
        tx_msg = Packet()
        tx_msg.data = msg_data
        tx_msg.dev_addr = self.gcu_addr
        tx_msg.is_broadcast = False
        self.tx_pub.publish(tx_msg)

    def pth_callback(self, msg):
        msg_data = [self.pth_code]
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
        msg_data += self._create_bytelist(ts, bytelen=8, vartype='d')
        msg_data += self._create_bytelist(msg.serial, bytelen=4, vartype='i')[0:2]
        msg_data += self._create_bytelist(msg.temp1)
        msg_data += self._create_bytelist(msg.temp2)
        msg_data += self._create_bytelist(msg.temp3)
        msg_data += self._create_bytelist(msg.pressure)
        msg_data += self._create_bytelist(msg.humidity)
        tx_msg = Packet()
        tx_msg.data = msg_data
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
