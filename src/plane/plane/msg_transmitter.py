#!/usr/bin/env python
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
from pydoc import locate

class MsgTransmitter(Node):
    def __init__(self):
        super().__init__("msg_transmitter")

        self.declare_parameters(namespace='',
                                parameters=[("sensor_topics", # assume we only have gps
                                             {
                                                "2":{
                                                         "topic": "gps_fix",
                                                         "msg_type": "sensor_msgs.msg.NavSatFix",
                                                         "attribute_order": ["longitude", "latitude", "altitude"],
                                                         "attributes": {
                                                             "longitude": [4, 'f'],
                                                             "latitude": [4, 'f'],
                                                             "altitude": [4, 'f']
                                                         }
                                                     }
                                             }
                                            ),
                                            ("gcu_addr", "13A20041D17945"),
                                            ("time_topic", "gps_time")
                                           ])
        sensor_topics = self.get_parameter("sensor_topics").value
        self.gcu_addr = self.get_parameter("gcu_addr").value

        # subscribe to each sensor, publish raw bytes to the transmitter
        self._timeref_sub = self.create_subscription(
            TimeReference,
            self.get_parameter("time_topic").value,
            self.timestamp_creator,
            1,
        )
        self.tx_pub = self.create_publisher(Packet, "transmit", 10)
        self._subs = []

        for sensor_code, topic in sensor_topics.items():
            self._subs.append(
                self.create_subscription(
                    locate(topic["msg_type"]),
                    topic["topic"],
                    self._generate_callback(sensor_code, topic)
                )
            )

        # setup management variables
        self.gps_code = "1"
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

    def _generate_callback(self, sensor_code, topic):
        def callback(self, msg):
            '''
            turn sensor message into a transmittable bytearray
            '''
            msg_data = [bytes(sensor_code, encoding="ascii")]
            sample_time = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
            )
            if sensor_code == self.gps_code:
                ts = sample_time
            elif self.rel_ts1:
                ts = self.interpolate_utc(
                    sample_time, self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2
                )
            else:
                return
            msg_data += self._create_bytelist(ts, bytelen=8, vartype="d")

            # construct your bytelist from ros msg attributes 
            for attr in topic["attribute_order"]:
                bytelen = topic["attributes"][attr][0]
                vartype = topic["attributes"][attr][1]
                msg_data += self._create_bytelist(getattr(msg, attr), bytelen=bytelen, vartype=vartype)

            tx_msg = Packet()
            tx_msg.data = msg_data
            tx_msg.dev_addr = self.gcu_addr
            tx_msg.is_broadcast = False
            self.tx_pub.publish(tx_msg)
        return callback

def main(args=None):
    rclpy.init(args=args)
    tx_pub = MsgTransmitter()
    rclpy.spin(tx_pub)
    tx_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
