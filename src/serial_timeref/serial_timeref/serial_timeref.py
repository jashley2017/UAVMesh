'''
ROS2 node for a time synchronized serial device. Contains a configuration driven abstract class
for string printing sensors.
'''
import threading
import time
import re
from serial import Serial
import importlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import TimeReference

class SerialSensor(Node):
    '''
    Description:
        Generic, configuration-driven serial sensor node that synchronizes its data using a
        timestamp.
    '''
    STR = 0
    INT = 1
    FLOAT = 2
    HEX = 3
    typecast_guide = {
        FLOAT: float,
        INT: int,
        HEX: lambda x: int(x, 16),
        STR: str
    }
    def __init__(self):
        super().__init__('serial')
        # see README for parameter description
        self.declare_parameter('sensor_top', 'serial1')
        self.declare_parameter('sensor_msg', "std_msgs.msg.String")
        self.declare_parameter('sensor_port', '/dev/ttyUSB0')
        self.declare_parameter('sensor_baud', 9600)
        self.declare_parameter('sensor_reg', None)
        self.declare_parameter('time_topic', 'gps_time')
        self.declare_parameter('sensor_key_types', None)

        # publisher setup
        pub_top = self.get_parameter('sensor_top').value
        pub_type = self.get_parameter('sensor_msg').value

        # the horrible things I do to make the code generic,
        # just import the msg package right please
        moduleobj = importlib.import_module('.'.join(pub_type.split('.')[:-1]))
        classname = pub_type.split('.')[-1]
        pub_msg_obj = getattr(moduleobj, classname)

        self.publisher = self.create_publisher(pub_msg_obj, pub_top, 10)

        # serial port setup
        ser_port = self.get_parameter('sensor_port').value
        ser_baud = self.get_parameter('sensor_baud').value
        ser_reg = self.get_parameter('sensor_reg').value
        ser_key_type = self.get_parameter('sensor_key_types').value
        ser_dev = Serial(ser_port, ser_baud)
        self.read_thread = threading.Thread(target=self.ser_callback,
                                            args=(ser_dev, ser_reg, ser_key_type, pub_msg_obj))

        # time reference subscriber setup
        self.rel_ts = [None, None]
        self.gps_ts = [None, None]
        self.lock = False
        self._subscription = self.create_subscription(
            TimeReference,
            self.get_parameter('time_topic').value,
            self.store_timestamp,
            1)

        self.running = True
        self.read_thread.start()

    def __del__(self):
        self.running = False
        self.read_thread.join()

    def store_timestamp(self, time_msg):
        '''
        Description:
            Converts Time message to a unix time float and stores it on the object the header
            stores the gps time and time_ref stores the local machine time. This
            aligns it with the NavSatFix format
        Parameters:
            - time_msg: TimeReference message for a UTC message
        '''
        self.lock = True
        # the time_ref holds the system time
        self.rel_ts[0] = self.rel_ts[1]
        self.rel_ts[1] = float(time_msg.time_ref.sec) + \
            float(time_msg.time_ref.nanosec) / 1000000000
        # the header hold the timestamp to the UTC value from the GPS
        self.gps_ts[0] = self.gps_ts[1]
        self.gps_ts[1] = float(time_msg.header.stamp.sec) + \
            float(time_msg.header.stamp.nanosec) / 1000000000
        self.lock = False

    @staticmethod
    def interpolate_utc(Cm, Cp1, Cp2, Tp1, Tp2):
        '''
        Description:
            Linearly interpolates the utc time of a given measurement from the gps time reference.
        Parameters:
            - Cm = time.time() value at the time of mth measurement
            - Cp1 = time.time() value at the time of the first ideal clock pulse
            - Cp2 = time.time() value at the time of the second ideal clock pulse
            - Tp1 = Time from the UBX message corresponding to first ideal clock pulse
            - Tp2 = Time from the UBX message corresponding to second ideal clock pulse
        '''
        return Tp1 + (Cm - Cp1)/(Cp2 - Cp1) * (Tp2 - Tp1)

    def ser_callback(self, ser, reg, key_types, msg_type):
        '''
        Description:
            Serial method that continuously listen on the port and converts the resulting string to
            a ROS message. Determined by the configuration.
        Parameters:
            - ser: Serial object
            - reg: regex to match a string to the message object
            - key_types: types of each of the matched attributes
            - msg_type: type of message to be published
        '''
        while rclpy.ok() and self.running:
            raw_msg = ser.readline().decode()
            if '\x00' in raw_msg:
                # skip erroneous characters
                continue
            # figuring out the gps time
            sample_time = time.time()
            while self.lock:
                time.sleep(0.05)
            if self.rel_ts[0]:
                timestamp = self.interpolate_utc(sample_time, self.rel_ts[0], self.rel_ts[1],
                                                 self.gps_ts[0], self.gps_ts[1])
            else:
                continue
            msg = msg_type()
            msg.header.stamp.sec = int(timestamp)
            msg.header.stamp.nanosec = int((timestamp - int(timestamp))*1000000000)

            # parsing all the components
            msg_match = re.match(reg, raw_msg)
            if msg_match:
                match_dict = msg_match.groupdict()
                for match_attr, match_val in match_dict:
                    if match_attr in msg.__slots__ and key_types.get(match_attr, False):
                        # this is the hacked way to get type conformity
                        # store the primitives as a key value pair that get typecast here
                        setattr(msg, match_attr,
                                self.typecast_guide[key_types[match_attr]](match_val))
                self.publisher.publish(msg)
                self.get_logger().info(f"Publishing serial message {raw_msg}")
            else:
                self.get_logger().warning(f"Got unmatched serial message {raw_msg}, \
                                          skipping publish.")

def main(args=None):
    '''very simple node spin up'''
    print("Launching")
    rclpy.init(args=args)
    ser_pub = SerialSensor()
    rclpy.spin(ser_pub)
    ser_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
