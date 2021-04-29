import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO
from pyubx2 import UBX_MSGIDS

# TODO: manage this better in setup.py
if __name__ == '__main__':
    from gpsreader import GPSReader
else:
    from .gpsreader import GPSReader

class Gps(Node):
    ON = b"\x00\x01\x01\x01\x00\x00"
    OFF = b"\x00\x00\x00\x00\x00\x00"
    PORT = "/dev/ttyS0"
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
        pub_top = self.get_parameter('gps_top').value
        self.publisher_ = self.create_publisher(String, pub_top, 10)
        # setup the gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(26, GPIO.IN, GPIO.PUD_UP)
        GPIO.add_event_detect(26, GPIO.RISING, 
            callback=self.timepulse_callback, bouncetime=50)

    def timepulse_callback(self, channel):
        gps_msg = String()
        try:
            ubx = self.ubp.read()
        except IOError:
            self.get_logger().warning("GPS disconnected. Attempting to reconnect.")
            self.ubp = GPSReader(self.port, self.baud, 
                    self.TIMEOUT, self.UBXONLY)
            return

        self.get_logger().info(f"Timepulse triggered.")
        pubbed = False
        while ubx and not pubbed:
            if (ubx.msg_cls + ubx.msg_id) == b"\x01\x07": # NAV_PVT
                gps_msg.data = str(ubx)
                self.publisher_.publish(gps_msg)
                self.get_logger().info(f"Publishing gps message: {gps_msg.data}")
                pubbed = True
            ubx = self.ubp.read()

def main(args=None):
    rclpy.init(args=args)
    gps_pub = Gps()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
