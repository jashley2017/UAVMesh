import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Pth(Node):
    def __init__(self):
        super().__init__('pth')
        self.declare_parameter('pth_top', 'pth')
        pub_top = self.get_parameter('pth_top').value
        self.publisher_ = self.create_publisher(String, pub_top, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.pth_callback)
        self.i = 0

    ''' 
    def timestamp_creator(self, msg):
        self.rel_ts = time.time()
        self.gps_ts = msg.data
    '''
    def pth_callback(self):
        # timestamp = self.gps_ts + (self.rel_ts - time.time())
        msg = String()
        msg.data = f"{self.i} Dummy publish from pth"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing pth: {self.i}")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    pth_pub = Pth()
    rclpy.spin(pth_pub)
    pth_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
