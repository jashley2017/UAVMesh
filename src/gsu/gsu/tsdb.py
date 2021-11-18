#!/usr/bin/env python
from influxdb import InfluxDBClient
import rclpy
import struct
from pydoc import locate
from networked_sensor.networked_sensor import Sensor # TODO: need sensor package
import datetime
import time
import re
from rclpy.node import Node
from xbee_interfaces.msg import Packet
from sensor_msgs.msg import TimeReference

class GroundStation(Node):
    def __init__(self):
        super().__init__('groundstation')
        self._subscription = self.create_subscription(
                Packet,
                'received',
                self.rx_callback,
                10)
        self.declare_parameter('outfile', '~/outfile.log')
        self.declare_parameter('time_topic', 'gps_time')
        self.declare_parameter('sensor_topics', {})

        self._subscription2 = self.create_subscription(
           TimeReference,
           self.get_parameter('time_topic').value,
           self.timestamp_creator,
        1)

        self.publisher = self.create_publisher(Packet, 'transmit', 10)
        self.rel_ts1 = None
        self.rel_ts2 = None
        self.gps_ts1 = None
        self.gps_ts2 = None

        user = 'ubuntu'
        password = 'piplane2021'
        dbname = 'flightdata'
        host = 'localhost'
        port = 8086
        self.client = InfluxDBClient(host, port, user, password, dbname)

        # sensor_topics

        self.sensor_topics = self.get_parameter('sensor_topics').value
        self.codes = {}
        self.specs = {}

    def timestamp_creator(self, time_msg):
        # the time_ref holds the system time
        self.rel_ts1 = self.rel_ts2
        self.rel_ts2 = float(time_msg.time_ref.sec) + float(time_msg.time_ref.nanosec) / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts1 = self.gps_ts2
        self.gps_ts2 = float(time_msg.header.stamp.sec) + float(time_msg.header.stamp.nanosec) / 1000000000

    @staticmethod
    def interpolate_utc(Cm, Cp1, Cp2, Tp1, Tp2):
        '''
        Linearly interpolates the utc time of a given measurement from the gps time reference.
        Cm = time.time() value at the time of mth measurement
        Cp1 = time.time() value at the time of the first ideal clock pulse
        Cp2 = time.time() value at the time of the second ideal clock pulse
        Tp1 = Time from the UBX message corresponding to first ideal clock pulse
        Tp2 = Time from the UBX message corresponding to second ideal clock pulse
        '''
        return Tp1 + (Cm - Cp1)/(Cp2 - Cp1) * (Tp2 - Tp1)

    @staticmethod
    def _unpack_bytelist(var, bytesize=4, vartype='f'):
        # unpacks a float from bytes[]
        return struct.unpack(vartype, struct.pack(str(bytesize) + 'c', *var))[0]

    def rx_callback(self, msg):
        if self.rel_ts1:
            # ts = self.interpolate_utc(time.time(), self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2)
            # self.get_logger().info(f"{time.time()} -> {ts}:\n [{self.rel_ts1}, {self.gps_ts1}]\n [{self.rel_ts2}, {self.gps_ts2}]")
            ts = time.time()
        else:
            return
        samples = []
        if msg.data[0] == b'0':
            # sensor spec
            spec_msg = str(struct.pack(str(len(msg.data)) + 'c', *msg.data), encoding='ascii')
            self.get_logger.info(f"Found sensor spec {spec_msg} from {msg.dev_addr}.")
            _, sensor_code, msg_type, topic = spec_msg.split(',')
            if not self.codes.get(msg.dev_addr, None):
                self.codes[msg.dev_addr] = [None]*255 # 255 max sensor type count
            self.codes[msg.dev_addr][sensor_code] = (topic, msg_type)
            if msg_type not in self.specs:
                self.specs[msg_type] = Sensor.parse_msg(locate(msg_type))
            tx_ack = Packet()
            tx_ack.data = [b'0', struct.pack('B', sensor_code)]
            tx_ack.dev_addr = msg.dev_addr
            self.publisher.publish(tx_ack)
        else:
            # sensor data
            code = struct.unpack('B', msg.data[0])[0]
            msg_stamp = _unpack_bytelist(msg.data[1:9], vartype='d')
            roundtrip_time = ts - msg_stamp

            topic, msg_type = self.codes[msg.dev_addr][code]
            spec = self.specs[msg_type]

            fields = {}
            tags = {"PlaneID": msg.dev_addr}
            byte_index = 9
            for field, field_type in spec:
                fields[field] = _unpack_bytelist(msg.data[byte_index: byte_index+struct.calcsize(field_type)], vartype=field_type)
                byte_index += struct.calcsize(field_type)
            fields["timelag"] = roundtrip_time
            samples =  [{
                    "measurement": topic,
                    "tags": tags,
                    "time": int(msg_stamp*1000),
                    "fields": fields
            }]
            self.client.write_points(samples, time_precision='ms')

def main(args=None):
    rclpy.init(args=args)
    gs = GroundStation()
    rclpy.spin(gs)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
