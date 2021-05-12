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
        self.gps_ts = datetime.datetime.now().strftime("%Y:%m:%d:%H:%M:%S:%f")
        self.read_thread = threading.Thread(target=self.pth_callback, args=(serial_port,))
        self.running = True
        self.read_thread.start()

    def __del__(self):
        self.running = False
        self.read_thread.join()

    def timestamp_creator(self, time_msg):
        self.rel_ts = float(f"{time_msg.time_ref.sec}.{time_msg.time_ref.nanosec}")
        # assuming source is string message of the UTC time as such:
        # yyyy:MM:dd:hh:mm:ss:nnnnnnnnn
        self.gps_ts = time_msg.source

    def pth_callback(self, ser):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        while rclpy.ok() and self.running:
            pth = ser.readline().decode()
            pth_time = time.time()
            pth_ts_str = self.gps_ts.split(":")
            year = int(pth_ts_str[0])
            month = int(pth_ts_str[1])
            day = int(pth_ts_str[2])
            hour = int(pth_ts_str[3])
            minute = int(pth_ts_str[4])
            second = int(pth_ts_str[5])
            usecond = int(int(pth_ts_str[6])/1000)
            utc = datetime.datetime(year, month, day, hour, minute, second, usecond)
            dt = pth_time - self.rel_ts
            ts = utc.timestamp() + dt
            adj_utc = datetime.datetime.fromtimestamp(ts)
            msg = String()
            msg.data = f"{adj_utc.strftime('%Y:%m:%d:%H:%M:%S:%f')}/{pth}"
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
