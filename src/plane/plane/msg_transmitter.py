#!/usr/bin/env python
import struct
import socket
import math
from pydoc import locate
import numpy as np
import rclpy
import yaml
import time
from networked_sensor.networked_sensor import Sensor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
# from environ_msgs.msg import Pth, iMET
from xbee_interfaces.msg import Packet
from sensor_msgs.msg import TimeReference, NavSatFix
from std_msgs.msg import String

class MsgTransmitter(Node):
    def __init__(self):
        super().__init__("msg_transmitter")

        self.declare_parameter("gcu_addr", "13A20041D17945")
        self.declare_parameter("time_topic", "gps_time")
        self.gcu_addr = self.get_parameter("gcu_addr").value

        # subscribe to each sensor, publish raw bytes to the transmitter
        self._timeref_sub = self.create_subscription(
            TimeReference,
            self.get_parameter("time_topic").value,
            self.timestamp_creator,
            1,
        )
        latching_qos = QoSProfile(depth=5, # QOS profile that queues message for subscriber
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.tx_pub = self.create_publisher(Packet, "transmit", latching_qos)
        self.create_subscription(Packet, "received", self.listen_incoming, 10)
        self.create_subscription(String, "sensor_descriptions", self._generate_callback, latching_qos)
        self._subs = []
        self.active_codes = []
        self.specs = {}

        # setup management variables
        self.rel_ts1 = None
        self.rel_ts2 = None
        self.gps_ts1 = None
        self.gps_ts2 = None
        self.code = 4 # starts past the reserved msgs
        # TODO is it important to get these from a config or is this good enough? for custom messages we could put a flag
        self.gps_ts_types = [Sensor.get_msg_fullpath(NavSatFix)]
        self.send_hostname()

    def timestamp_creator(self, time_msg):
        """
        store the relationship between gps time and local time to the object
        """
        # the time_ref holds the system time
        new_rel_ts = (
            float(time_msg.time_ref.sec) + float(time_msg.time_ref.nanosec) / 1000000000
        )
        if self.rel_ts2 is not None:
            if math.isclose(self.rel_ts2, new_rel_ts): 
                return
        self.rel_ts1 = self.rel_ts2
        self.rel_ts2 = new_rel_ts
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts1 = self.gps_ts2
        self.gps_ts2 = (
            float(time_msg.header.stamp.sec)
            + float(time_msg.header.stamp.nanosec) / 1000000000
        )

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
    def _create_bytelist(*var, vartype="f"):
        return list(struct.unpack(str(struct.calcsize(vartype)) + "c", struct.pack(vartype, *var)))

    def send_spec(self, sensor_code, msg_type, topic):
        data = [struct.pack("c", bytes(c, encoding="ascii")) for c in f"0,{sensor_code},{msg_type},{topic}"]
        self._tx(data, self.gcu_addr)
        self.get_logger().info(f"Transmitting topic: {topic}.")
        # self.get_logger().info(f"sending specification: 0,{sensor_code},{msg_type},{topic}")

    def send_hostname(self):
        hostname = socket.gethostname()
        data = [struct.pack("c", bytes(c, encoding="ascii")) for c in f"2,{hostname}"]
        self._tx(data, self.gcu_addr)

    def _tx(self, data, dev_addr):
        msg = Packet()
        msg.data = data
        msg.dev_addr = dev_addr
        msg.is_broadcast = False
        self.tx_pub.publish(msg)

    def _generate_callback(self, desc):
        '''
        Parameters:
            desc <- sensor description generated automatically
            by a publisher inheriting the Sensor class
        Description:
            this method takes the publisher description and forms
            it into the appropriate listener for the plane manager
        '''
        sensor_desc = yaml.safe_load(desc.data)
        msg_type, topic = sensor_desc['subscribe']
        conversions = sensor_desc['attributes']

        sensor_code = self.code
        self.code += 1

        self.specs[sensor_code] = (msg_type, topic)
        self.send_spec(sensor_code, msg_type, topic)
        # Send the new sensor code as a message to GSU

        if msg_type in self.gps_ts_types: # if the message timestamp is a gps timestamp
            def callback(msg):
                '''
                turn sensor message into a transmittable bytearray
                '''
                if sensor_code in self.active_codes:
                    msg_data = [struct.pack('B', sensor_code)]
                    ts = (
                        float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
                    )
                    msg_data += self._create_bytelist(ts, vartype="d")

                    # construct your bytelist from ros msg attributes
                    for attr, vartype in conversions:
                        if isinstance(getattr(msg,attr), type(np.array([]))):
                            msg_data += self._create_bytelist(*getattr(msg, attr).tolist(), vartype=vartype)
                        else:
                            msg_data += self._create_bytelist(getattr(msg, attr), vartype=vartype)

                    tx_msg = Packet()
                    tx_msg.data = msg_data
                    tx_msg.dev_addr = self.gcu_addr
                    tx_msg.is_broadcast = False
                    self.tx_pub.publish(tx_msg)
                else:
                    self.send_spec(sensor_code, *self.specs[sensor_code])
        else:
            def callback(msg):
                '''
                turn sensor message into a transmittable bytearray
                '''
                if sensor_code in self.active_codes:
                    msg_data = [struct.pack('B', sensor_code)]

                    sample_time = (
                        float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1000000000
                    )
                    if not self.rel_ts1: 
                        return
                    ts = self.interpolate_utc(
                        sample_time, self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2
                    )

                    # self.get_logger().info(f"{time.time()} -> {ts}:\n [{self.rel_ts1}, {self.gps_ts1}]\n [{self.rel_ts2}, {self.gps_ts2}]")
                    msg_data += self._create_bytelist(ts, vartype="d")

                    # construct your bytelist from ros msg attributes
                    for attr, vartype in conversions:
                        if isinstance(getattr(msg,attr), type(np.array([]))):
                            msg_data += self._create_bytelist(*getattr(msg, attr).tolist(), vartype=vartype)
                        else:
                            msg_data += self._create_bytelist(getattr(msg, attr), vartype=vartype)
                    tx_msg = Packet()
                    tx_msg.data = msg_data
                    tx_msg.dev_addr = self.gcu_addr
                    tx_msg.is_broadcast = False
                    self.tx_pub.publish(tx_msg)
                else:
                    self.send_spec(sensor_code, *self.specs[sensor_code])
        self._subs.append(self.create_subscription(locate(msg_type), topic, callback, 1))

    def listen_incoming(self, rx_msg):
        self.get_logger().info(f"received message {rx_msg.data}")
        if rx_msg.data[0] == b'0':
            # receiving an ack to a sensor
            msg = str(struct.pack(str(len(rx_msg.data)) + 'c', *rx_msg.data), encoding='ascii')
            good_sensor_code = int.from_bytes(b''.join(rx_msg.data[2:]), 'big')
            self.active_codes.append(good_sensor_code)
            return
        if rx_msg.data[0] == b'1': # gsu doesnt know sensor
            msg = str(struct.pack(str(len(rx_msg.data)) + 'c', *rx_msg.data), encoding='ascii')
            sensor_code = int.from_bytes(b''.join(rx_msg.data[2:]), 'big')
            self.send_spec(sensor_code, *self.specs[sensor_code])
            return
        if rx_msg.data[0] == b'3': # gsu doesnt know plane
            self.send_hostname()
            return

def main(args=None):
    rclpy.init(args=args)
    tx_pub = MsgTransmitter()
    rclpy.spin(tx_pub)
    tx_pub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
