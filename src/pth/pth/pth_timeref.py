import rclpy
from rclpy.node import Node
from environ_msgs.msg import Pth
from sensor_msgs.msg import TimeReference
import threading
import time
import re
import datetime
from serial import Serial

class PthProbe(Node):
    def __init__(self):
        super().__init__('pth')
        self.declare_parameter('pth_top', 'pth')
        # defaults to a virtual port initialize by socat
        # socat -d -d pty,raw,echo=0 pty,raw,echo=0
        self.declare_parameter('pth_port', '/dev/pts/2')
        self.declare_parameter('pth_baud', 9600)
        self.declare_parameter('time_topic', 'gps_time')
        pub_top = self.get_parameter('pth_top').value
        self.publisher_ = self.create_publisher(Pth, pub_top, 10)
        serial_port = Serial(self.get_parameter('pth_port').value, 
                self.get_parameter('pth_baud').value)
        self._subscription = self.create_subscription(
           TimeReference, 
           self.get_parameter('time_topic').value, 
           self.timestamp_creator,
           1)
        self.rel_ts = time.time()
        self.gps_ts = time.time()
        self.read_thread = threading.Thread(target=self.pth_callback, args=(serial_port,))
        self.running = True
        self.read_thread.start()
        self.lock = False

    def __del__(self):
        self.running = False
        self.read_thread.join()

    def timestamp_creator(self, time_msg):
        # TODO: this is probably backwards from what it should be, but it lines up with NavSatFix
        self.lock = True
        # the time_ref holds the system time 
        self.rel_ts = float(time_msg.time_ref.sec) + float(time_msg.time_ref.nanosec) / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts = float(time_msg.header.stamp.sec) + float(time_msg.header.stamp.nanosec) / 1000000000
        self.lock = False

    def pth_callback(self, ser):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        pth_reg = r"\$UKPTH,(?P<serial>[0-9A-F]{4}),(?P<pressure_val>\d*\.?\d*),(?P<pressure_unit>[A-Za-z]+),(?P<temp1_val>\d*\.?\d*),(?P<temp1_unit>[A-Za-z]+),(?P<temp2_val>\d*\.?\d*),(?P<temp2_unit>[A-Za-z]+),(?P<hum_val>\d*\.?\d*),(?P<hum_unit>[^0-9,]*),(?P<temp3_val>\d*\.?\d*),(?P<temp3_unit>[A-Za-z]+)"
        while rclpy.ok() and self.running:
            pth = ser.readline().decode()
            if '\x00' in pth:
                continue
            # figuring out the gps time
            pth_time = time.time()
            while self.lock:
                time.sleep(0.05)
            dt = pth_time - self.rel_ts
            ts = self.gps_ts + dt
            msg = Pth()
            msg.header.stamp.sec = int(ts)
            msg.header.stamp.nanosec = int((ts - int(ts))*1000000000)

            # parsing all the components
            pth_match = re.match(pth_reg, pth)
            if pth_match:
                match_dict = pth_match.groupdict()
                msg.serial = int(match_dict['serial'], 16) 
                msg.temp1 = float(match_dict['temp1_val'])
                msg.temp2 = float(match_dict['temp2_val'])
                msg.temp3 = float(match_dict['temp3_val'])
                msg.pressure = float(match_dict['pressure_val'])
                msg.humidity = float(match_dict['hum_val'])
                self.publisher_.publish(msg)
                self.get_logger().info(f"Publishing pth {msg.serial}")

def main(args=None):
    rclpy.init(args=args)
    pth_pub = PthProbe()
    rclpy.spin(pth_pub)
    pth_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
