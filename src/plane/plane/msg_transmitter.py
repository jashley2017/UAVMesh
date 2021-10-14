import struct
import re
import datetime
import rclpy
from rclpy.node import Node
from environ_msgs.msg import Pth
from sensor_msgs.msg import NavSatFix
from xbee_interfaces.msg import Packet
from sensor_msgs.msg import TimeReference
from uldaq_msgs.msg import Measurement


class MsgTransmitter(Node):
    def __init__(self):
        super().__init__("msg_transmitter")

        self.declare_parameter("pth_top", "pth_msg")
        self.declare_parameter("daq_top", "uldaq_measurement")
        self.declare_parameter("gps_top", "gps_fix")
        # XBee node MAC address corresponding to the node on the ground station
        self.declare_parameter("gcu_addr", "13A20041D17945")
        self.declare_parameter("time_topic", "gps_time")

        pth_top = self.get_parameter("pth_top").value
        daq_top = self.get_parameter("daq_top").value
        gps_top = self.get_parameter("gps_top").value
        self.gcu_addr = self.get_parameter("gcu_addr").value

        # subscribe to each sensor, publish raw bytes to the transmitter
        self._timeref_sub = self.create_subscription(
            TimeReference,
            self.get_parameter("time_topic").value,
            self.timestamp_creator,
            1,
        )
        self._gps_sub = self.create_subscription(
            NavSatFix, gps_top, self.gps_callback, 1
        )
        self._pth_sub = self.create_subscription(Pth, pth_top, self.pth_callback, 1)
        self._daq_sub = self.create_subscription(Measurement, daq_top, self.daq_callback, 1)
        self.tx_pub = self.create_publisher(Packet, "transmit", 10)

        # setup management variables
        self.gps_code = b"1"
        self.pth_code = b"2"
        self.daq_code = b"3"

        self.lock = False
        self.rel_ts1 = None
        self.rel_ts2 = None
        self.gps_ts1 = None
        self.gps_ts2 = None

    def timestamp_creator(self, time_msg):
        """
        store the relationship between gps time and local time to the object
        """
        self.lock = True
        # the time_ref holds the system time
        self.rel_ts1 = self.rel_ts2
        self.rel_ts2 = (
            float(time_msg.time_ref.sec) + float(time_msg.time_ref.nanosec) / 1000000000
        )
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts1 = self.gps_ts2
        self.gps_ts2 = (
            float(time_msg.header.stamp.sec)
            + float(time_msg.header.stamp.nanosec) / 1000000000
        )
        self.lock = False

    @staticmethod
    def interpolate_utc(Cm, Cp1, Cp2, Tp1, Tp2):
        """
        Linearly interpolates the utc time of a given measurement from the gps time reference.
        Cm = time.time() value at the time of mth measurement
        Cp1 = time.time() value at the time of the first ideal clock pulse
        Cp2 = time.time() value at the time of the second ideal clock pulse
        Tp1 = Time from the UBX message corresponding to first ideal clock pulse
        Tp2 = Time from the UBX message corresponding to second ideal clock pulse
        """
        return Tp1 + (Cm - Cp1) / (Cp2 - Cp1) * (Tp2 - Tp1)


    @staticmethod
    def _create_bytelist(var, bytelen=4, vartype="f"):
        return list(struct.unpack(str(bytelen) + "c", struct.pack(vartype, var)))

    def gps_callback(self, msg):
        '''
        turn gps message into a bytestring
        msg is NavSatFix type
        '''
        msg_data = [self.gps_code]
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
        msg_data += self._create_bytelist(ts, bytelen=8, vartype="d")
        msg_data += self._create_bytelist(msg.latitude)
        msg_data += self._create_bytelist(msg.longitude)
        msg_data += self._create_bytelist(msg.altitude)
        tx_msg = Packet()
        tx_msg.data = msg_data
        tx_msg.dev_addr = self.gcu_addr
        tx_msg.is_broadcast = False
        self.tx_pub.publish(tx_msg)

    def pth_callback(self, msg):
        '''
        turn the pth message into a bytestring
        msg type is Pth
        '''
        msg_data = [self.pth_code]
        pth_time = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
        )
        if self.rel_ts1:
            ts = self.interpolate_utc(
                pth_time, self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2
            )
        else:
            return
        msg_data += self._create_bytelist(ts, bytelen=8, vartype="d")
        msg_data += self._create_bytelist(msg.serial, bytelen=4, vartype="i")[0:2]
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

    def daq_callback(self, msg):
        msg_data = [self.daq_code]
        daq_time = (
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
        )
        if self.rel_ts1:
            ts = self.interpolate_utc(
                daq_time, self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2
            )
        else:
            return
        msg_data += self._create_bytelist(ts, bytelen=8, vartype="d")
        for sig in msg.data:
            msg_data += self._create_bytelist(sig, bytelen=8, vartype="f")
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


if __name__ == "__main__":
    main()
