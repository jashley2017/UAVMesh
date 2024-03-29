#!/usr/bin/env python
import struct
from pydoc import locate
import time
import rclpy
from rclpy.node import Node
from influxdb import InfluxDBClient
from sensor_msgs.msg import TimeReference
from networked_sensor.networked_sensor import Sensor
from xbee_interfaces.msg import Packet

class GroundStation(Node):
    tag_fields = ["serial"] # fields in data that indicate a category of data (i.e. serial number)
    def __init__(self):
        super().__init__('groundstation')
        self._subscription = self.create_subscription(
                Packet,
                'received',
                self.rx_callback,
                10)
        self.declare_parameter('outfile', '~/outfile.log')
        self.declare_parameter('time_topic', 'gps_time')

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

        self.plane_names = {}
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
    def _unpack_bytelist(var, vartype='f'):
        # unpacks a float from bytes[]
        decompressed = struct.unpack(vartype, struct.pack(str(struct.calcsize(vartype)) + 'c', *var))
        if len(decompressed) == 1:
            return decompressed[0]
        return decompressed

    def rx_callback(self, msg):
        if self.rel_ts1:
            ts = self.interpolate_utc(time.time(), self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2)
            # self.get_logger().info(f"{time.time()} -> {ts}:\n [{self.rel_ts1}, {self.gps_ts1}]\n [{self.rel_ts2}, {self.gps_ts2}]")
        else:
            return
        samples = []
        if msg.data[0] == b'0': # sensor spec
            spec_msg = str(struct.pack(str(len(msg.data)) + 'c', *msg.data), encoding='ascii')
            # self.get_logger().info(f"Found sensor spec {spec_msg} from {msg.dev_addr}.")
            _, sensor_code, msg_type, topic = spec_msg.split(',')
            if not self.codes.get(msg.dev_addr, None):
                self.codes[msg.dev_addr] = [None]*255 # 255 max sensor type count
            self.codes[msg.dev_addr][int(sensor_code)] = (topic, msg_type)
            if msg_type not in self.specs:
                self.specs[msg_type], _ = Sensor.generate_struct_spec_for(locate(msg_type))
                # self.get_logger().info(f"resulting spec: {self.specs[msg_type]}")
            self.sensor_acknowledge(sensor_code, msg.dev_addr)
        elif msg.data[0] == b'2': # register plane
            plane_msg = str(struct.pack(str(len(msg.data)) + 'c', *msg.data), encoding='ascii')
            _, plane_hostname = plane_msg.split(',')
            self.plane_names[msg.dev_addr] = plane_hostname
            if not self.codes.get(msg.dev_addr, None):
                self.codes[msg.dev_addr] = [None]*255 # 255 max sensor type count
        else: # sensor data
            # sensor data
            code = struct.unpack('B', msg.data[0])[0]
            msg_stamp = self._unpack_bytelist(msg.data[1:9], vartype='d')
            roundtrip_time = max([ts - msg_stamp, 0.]) # dont let the negatives pollute too hard

            if self.plane_names.get(msg.dev_addr, None) is None: # unknown plane
                self.unknown_plane(msg.dev_addr)
                return
            if self.codes[msg.dev_addr][code] is None: # unknown sensor
                self.unknown_sensor(code, msg.dev_addr)
                return

            topic, msg_type = self.codes[msg.dev_addr][code]
            # self.get_logger().info(f"Received topic: {topic}")
            # self.get_logger().info(f"Time recv: {msg_stamp}, {ts}")
            spec = self.specs[msg_type]
            fields = {}
            tags = {"PlaneID": self.plane_names[msg.dev_addr]}
            byte_index = 9
            for field, field_type in spec:
                unpacked = self._unpack_bytelist(msg.data[byte_index: byte_index+struct.calcsize(field_type)], vartype=field_type)
                if field in self.tag_fields: # this does not allow for a tag to be enumerated
                    category = tags
                else:
                    category = fields
                if isinstance(unpacked, tuple):
                    for i, subfield in enumerate(unpacked):
                        category[field + str(i)] = subfield
                else:
                    category[field] = unpacked
                byte_index += struct.calcsize(field_type)
            fields["timelag"] = roundtrip_time
            samples =  [{
                    "measurement": topic,
                    "tags": tags,
                    "time": int(msg_stamp*1000),
                    "fields": fields
            }]
            # self.get_logger().info(f"logging: {samples}")
            self.client.write_points(samples, time_precision='ms')

    # protocol formatting helper functions
    def unknown_plane(self, dev_addr):
        self.get_logger().warn("Got message from unknown plane, asking for hostname.")
        data = [b'3']
        self._tx(data, dev_addr)

    def unknown_sensor(self, code, dev_addr):
        self.get_logger().warn(f"Received unknown sensor code from {self.plane_names.get(dev_addr, 'Unknown plane')}. Asking for retransmit.")
        data = [b'1', b',', struct.pack('B', int(code))]
        self._tx(data, dev_addr)

    def sensor_acknowledge(self, code, dev_addr):
        data = [b'0', b',', struct.pack('B', int(code))]
        self._tx(data, dev_addr)

    def _tx(self, data, dev_addr):
        msg = Packet()
        msg.data = data
        msg.dev_addr = dev_addr
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    gs = GroundStation()
    rclpy.spin(gs)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
