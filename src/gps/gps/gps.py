import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

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
        # setup the GPS
        super().__init__('gps')
        self.ubp = GPSReader(self.PORT, self.BAUDRATE, 
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
        gps_msg.data = str(self.ubp.read())
        self.publisher_.publish(gps_msg)
        self.get_logger().info(f"Publishing gps message: {gps_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    gps_pub = Gps()
    rclpy.spin(gps_pub)
    gps_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
