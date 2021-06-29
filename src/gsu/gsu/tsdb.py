from influxdb import InfluxDBClient
import rclpy
import struct
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

        self._subscription = self.create_subscription(
           TimeReference, 
           self.get_parameter('time_topic').value, 
           self.timestamp_creator,
        1)
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
        self.gps_code = b'1'
        self.pth_code = b'2'
        self.lock = False

    def timestamp_creator(self, time_msg):
        self.lock = True
        # the time_ref holds the system time 
        self.rel_ts1 = self.rel_ts2
        self.rel_ts2 = float(time_msg.time_ref.sec) + float(time_msg.time_ref.nanosec) / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts1 = self.gps_ts2
        self.gps_ts2 = float(time_msg.header.stamp.sec) + float(time_msg.header.stamp.nanosec) / 1000000000
        self.lock = False

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
        while self.lock:
            time.sleep(0.05)
        if self.rel_ts1:
            ts = self.interpolate_utc(time.time(), self.rel_ts1, self.rel_ts2, self.gps_ts1, self.gps_ts2)
            # self.get_logger().info(f"{time.time()} -> {ts}:\n [{self.rel_ts1}, {self.gps_ts1}]\n [{self.rel_ts2}, {self.gps_ts2}]")
        else:
            return
        samples = []
        if msg.data[0] == self.gps_code or msg.data[0] == self.pth_code:
            msg_stamp = self._unpack_bytelist(msg.data[1:9], bytesize=8, vartype='d')
            # self.get_logger().info(f"current timestamps: MSG:{msg_stamp}")
            roundtrip_time = ts - msg_stamp
            if roundtrip_time > 0:
                samples = [
                        {
                            "measurement": "time_lag", 
                            "tags": {
                                "host": "groundstation",
                                "region": "us-east"
                                },
                            "time": int(ts*1000), # ms precision 
                            "fields": {
                                "Time_Difference": roundtrip_time,
                                }
                        }
                ]
            if msg.data[0] == self.pth_code:
                samples.append(
                        {
                            "measurement": "pth",
                            "tags": {
                                "host": "groundstation",
                                "region": "us-east"
                                },
                            "time": int(msg_stamp*1000),
                            "fields": {
                                # TODO: units
                                "PlaneID": msg.dev_addr,
                                "Serial": struct.unpack('i', struct.pack('4c', *msg.data[9:11], b'\x00', b'\x00'))[0],
                                "Temperature1": self._unpack_bytelist(msg.data[11:15]),
                                "Temperature2": self._unpack_bytelist(msg.data[15:19]),
                                "Temperature3": self._unpack_bytelist(msg.data[19:23]),
                                "Pressure": self._unpack_bytelist(msg.data[23:27]), 
                                "Humidity": self._unpack_bytelist(msg.data[27:31])
                            }
                        }
                )
            else:
                samples.append(
                        {
                            "measurement": "gps",
                            "tags": {
                                "host": "groundstation",
                                "region": "us-east"
                                },
                            "time": int(msg_stamp*1000),
                            "fields": {
                                "PlaneID": msg.dev_addr,
                                "Latitude": self._unpack_bytelist(msg.data[9:13]),
                                "Longitude": self._unpack_bytelist(msg.data[13:17]),
                                "Altitude": self._unpack_bytelist(msg.data[17:21])
                            }
                        }
                )
        self.client.write_points(samples, time_precision='ms')

def main(args=None):
    rclpy.init(args=args)
    gs = GroundStation()
    rclpy.spin(gs)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
