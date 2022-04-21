#!/usr/bin/env python
import struct
import os
import datetime 
from pydoc import locate
import yaml
from networked_sensor.networked_sensor import Sensor
import rclpy
from rclpy.node import Node
# from environ_msgs.msg import Pth, iMET
from xbee_interfaces.msg import Packet
from sensor_msgs.msg import TimeReference, NavSatFix
from std_msgs.msg import String

class MsgLogger(Node):
    def __init__(self):
        super().__init__("msg_logger")
        self.declare_parameter("time_topic", "gps_time")
        self.declare_parameter("logdir", f"{os.environ['HOME']}")
        self.declare_parameter("logall", False)
        self._timeref_sub = self.create_subscription(
            TimeReference,
            self.get_parameter("time_topic").value,
            self.timestamp_creator,
            1,
        )
        self.spec_sub = self.create_subscription(String, "sensor_descriptions", self._generate_callback)

        self.rel_ts1 = None
        self.rel_ts2 = None
        self.gps_ts1 = None
        self.gps_ts2 = None

        self.gps_ts_types = [Sensor.get_msg_fullpath(NavSatFix)]
        self.logpath = datetime.datetime.now().strftime(f"{self.get_parameter('logdir').value}/%Y%m%d_%H%M%S")
        if not os.path.isdir(self.logpath):
            os.mkdir(self.logpath)
        self._subs = []

    def timestamp_creator(self, time_msg):
        """
        store the relationship between gps time and local time to the object
        """
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
        if sensor_desc['log'] or self.get_parameter('logall').value:
            def _callback(msg):
                # construct your bytelist from ros msg attributes
                msg_data = ""
                for attr, _ in conversions:
                    msg_data += str(getattr(msg, attr))
                logname = f"{self.logpath}/{topic}.log"
                if not os.path.isfile(logname):
                    with open(logname,"a+") as file:
                        header = ''.join([attr for attr, _ in conversions])
                        file.write(header)
                        file.write("\n")
                        file.write(msg_data)
                        file.write("\n")
                else:
                    with open(logname,"a+") as file:
                        file.write(msg_data)
                        file.write("\n")
            self._subs.append(self.create_subscription(locate(msg_type), topic, _callback, 1))

def main(args=None):
    rclpy.init(args=args)
    node = MsgLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()