from influxdb import InfluxDBClient
import rclpy
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
        self.rel_ts = None
        self.gps_ts = None

        user = 'ubuntu'
        password = 'piplane2021'
        dbname = 'flightdata'
        host = 'localhost'
        port = 8086
        self.client = InfluxDBClient(host, port, user, password, dbname)

    def timestamp_creator(self, time_msg):
        # the time_ref holds the system time 
        self.rel_ts = time_msg.time_ref.sec + time_msg.time_ref.nanosec / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts = time_msg.header.stamp.sec + time_msg.header.stamp.nanosec / 1000000000

    def rx_callback(self, msg):
        ts = time.time()
        pth_reg = r"(?P<timestamp>\d+)\/(?P<pressure_val>\d*\.?\d*),(?P<pressure_unit>[A-Za-z]+),(?P<temp1_val>\d*\.?\d*),(?P<temp1_unit>[A-Za-z]+),(?P<temp2_val>\d*\.?\d*),(?P<temp2_unit>[A-Za-z]+),(?P<hum_val>\d*\.?\d*),(?P<hum_unit>[^0-9,]*),(?P<temp3_val>\d*\.?\d*),(?P<temp3_unit>[A-Za-z]+)"
        gps_reg = r"(?P<timestamp>\d+)\/(?P<lat>\d*\.?\d*),(?P<lon>-?\d*\.?\d*),(?P<alt>-?\d*\.?\d*)"
        if self.rel_ts:
            dt = time.time() - self.rel_ts
            ts = self.gps_ts + dt
        pth_match = re.match(pth_reg, msg.data)
        gps_match = re.match(gps_reg, msg.data)
        samples = []
        if pth_match or gps_match:
            if pth_match:
                match_dict = pth_match.groupdict()
            else:
                match_dict = gps_match.groupdict()
            timestr = match_dict['timestamp']
            timeobj = datetime.datetime.strptime(timestr, '%Y%m%d%H%M%S%f')
            msg_stamp = datetime.datetime.timestamp(timeobj)
            roundtrip_time = ts - msg_stamp
            samples = [
                    {
                        "measurement": "time_lag", 
                        "tags": {
                            "host": "groundstation",
                            "region": "us-east"
                            },
                        "time": int(ts*1000), # ms precision 
                        "fields": {
                            "Time_Difference": roundtrip_time
                            }
                    }
            ]
            if pth_match:
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
                                "Temperature1": match_dict['temp1_val'],
                                "Temperature2": match_dict['temp2_val'],
                                "Temperature3": match_dict['temp3_val'],
                                "Pressure": match_dict['pressure_val'],
                                "Humidity": match_dict['hum_val']
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
                                "Latitude": match_dict['lat'],
                                "Longitude": match_dict['lon'],
                                "Altitude": match_dict['alt'],
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
