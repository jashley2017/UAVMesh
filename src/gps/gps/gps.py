#!/usr/bin/env python
import datetime
import time
import rclpy
from rclpy.node import Node
from networked_sensor.networked_sensor import Sensor
from rclpy.time import Time
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
import RPi.GPIO as GPIO
from pyubx2 import UBX_MSGIDS

# TODO: manage this better in setup.py
if __name__ == '__main__':
    from gpsreader import GPSReader
else:
    from .gpsreader import GPSReader

SERVICE_GPS = 1
SERVICE_GLONASS = 2
SERVICE_COMPASS = 4
SERVICE_GALILEO = 8

class Gps(Sensor):
    ON = b"\x00\x01\x01\x01\x00\x00"
    OFF = b"\x00\x00\x00\x00\x00\x00"
    PORT = "/dev/ttyACM0"
    BAUDRATE = 9600
    TIMEOUT = 1
    UBXONLY = False
    def __init__(self):
        super().__init__('gps')
        # setup the GPS
        self.declare_parameter('gps_baud', self.BAUDRATE)
        self.declare_parameter('gps_port', self.PORT)
        self.baud = self.get_parameter('gps_baud').value
        self.port = self.get_parameter('gps_port').value
        self.ubp = GPSReader(self.port, self.baud,
                self.TIMEOUT, self.UBXONLY)
        if not self.ubp.connect():
            # TODO: raise custom error
            self.get_logger().error(f"GPS Failed to connect on given port. Aborting")
            return
        self.ubp.config_timepulse_badly()
        self.ubp.config_msg(self.ON)
        # setup the ROS
        self.declare_parameter('gps_top', 'gps')
        self.declare_parameter('time_top', 'gps_time')
        pub_top = self.get_parameter('gps_top').value
        time_top = self.get_parameter('time_top').value
        self.fix_pub = self.create_publisher(NavSatFix, pub_top, 10)
        # we dont want the timereferences transmitted so it needs to be local (see networked_sensor.Sensor)
        self.time_pub = self.create_local_publisher(TimeReference, time_top, 10)
        # setup the gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(26, GPIO.IN, GPIO.PUD_UP)
        GPIO.add_event_detect(26, GPIO.RISING,
            callback=self.timepulse_callback, bouncetime=50)

    def timepulse_callback(self, channel):
        gps_msg = NavSatFix()
        timeref_msg = TimeReference()
        msg_hdr = Header()
        system_time = self.get_clock().now().to_msg()
        msg_hdr.frame_id = 'base_link' # center of the plane
        try:
            ubx = self.ubp.read()
        except IOError:
            self.get_logger().warning("GPS disconnected. Attempting to reconnect.")
            self.ubp = GPSReader(self.port, self.baud, 
                    self.TIMEOUT, self.UBXONLY)
            return
        while ubx:
            if (ubx.msg_cls + ubx.msg_id) == b"\x01\x07": # NAV_PVT
                # <UBX(NAV-PVT, iTOW=16:50:32, year=2015, month=10, day=25, hour=16, min=50, second=48, valid=b'\xf0', tAcc=4294967295, nano=0, fixType=0, flags=b'\x00', flags2=b'$', numSV=0, lon=0, lat=0, height=0, hMSL=-17000, hAcc=4294967295, vAcc=4294967295, velN=0, velE=0, velD=0, gSpeed=0, headMot=0, sAcc=20000, headAcc=18000000, pDOP=9999, reserved1=65034815406080, headVeh=0, magDec=0, magAcc=0)>

                msg_hdr.stamp = self._gen_timestamp_from_utc(ubx)

                fix_stat = NavSatStatus()

                if ubx.fixType == 0:
                    self.get_logger().warning(f"No fix yet.")
                    break

                fix_stat.service = SERVICE_GPS

                gps_msg.status = fix_stat
                gps_msg.header = msg_hdr
                gps_msg.latitude = float(ubx.lat)/10000000
                gps_msg.longitude = float(ubx.lon)/10000000
                gps_msg.altitude = float(ubx.height)/1000

                timeref_msg.header = msg_hdr
                timeref_msg.time_ref = system_time
                timeref_msg.source = "GPS"

                self.fix_pub.publish(gps_msg)
                self.time_pub.publish(timeref_msg)

                return
            else:
                self.get_logger().info(f"Other GPS MSG: {(ubx.msg_cls + ubx.msg_id)}")
                ubx = self.ubp.read()

    def _gen_timestamp_from_utc(self, ubx):
        second = ubx.second
        usecond = ubx.nano/1000
        subtract_sec = False
        if usecond < 0:
            usecond = 1000000 + usecond
            subtract_sec = True
        utc = datetime.datetime(ubx.year, ubx.month, ubx.day, ubx.hour, ubx.min, second, int(usecond))
        gps_time = utc.timestamp()
        if subtract_sec:
            gps_time -= 1
        gps_stamp = self.get_clock().now().to_msg()
        gps_stamp.sec = int(gps_time)
        gps_stamp.nanosec = int((gps_time - int(gps_time))*1000000000)
        return gps_stamp

def main(args=None):
    rclpy.init(args=args)
    gps_pub = Gps()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
