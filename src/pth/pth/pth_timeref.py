import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial import Serial
from sensor_msgs.msg import TimeReference
import threading
import time
import datetime

class Pth(Node):
    def __init__(self):
        super().__init__('pth')
        self.declare_parameter('pth_top', 'pth')
        # defaults to a virtual port initialize by socat
        # socat -d -d pty,raw,echo=0 pty,raw,echo=0
        self.declare_parameter('pth_port', '/dev/pts/2')
        self.declare_parameter('pth_baud', 9600)
        self.declare_parameter('time_topic', 'gps_time')
        pub_top = self.get_parameter('pth_top').value
        self.publisher_ = self.create_publisher(String, pub_top, 10)
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

    def __del__(self):
        self.running = False
        self.read_thread.join()

    def timestamp_creator(self, time_msg):
        # TODO: this is probably backwards from what it should be, but it lines up with NavSatFix
        # the time_ref holds the system time 
        self.rel_ts = time_msg.time_ref.sec + time_msg.time_ref.nanosec / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts = time_msg.header.stamp.sec + time_msg.header.stamp.nanosec / 1000000000

    def pth_callback(self, ser):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        while rclpy.ok() and self.running:
            pth = ser.readline().decode()
            if '\x00' in pth:
                continue
            pth_time = time.time()
            dt = pth_time - self.rel_ts
            ts = self.gps_ts + dt
            adj_utc = datetime.datetime.fromtimestamp(ts)
            msg = String()
            msg.data = f"{adj_utc.strftime('%Y%m%d%H%M%S%f')}/{pth[12:-5]}"   
            # msg.data = f"{pth[:-2]}"
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing pth: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    pth_pub = Pth()
    rclpy.spin(pth_pub)
    pth_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
