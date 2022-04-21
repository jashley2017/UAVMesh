import struct
import time
import re
from std_msgs.msg import String
from environ_msgs.msg import Pth
import rclpy
from rclpy.node import Node

class Sensor(Node):
    MAX_PACKET = 70
    def __init__(self, node_name):
        time.sleep(5) # TODO : do better. this ensures the subscribers spin up
        super().__init__(node_name)
    def create_publisher(self, msg_type, topic, queue=10):
        '''overrides the base create publisher to send a sensor description before the information comes back'''
        # parameter_events is a background topic
        if topic != '/parameter_events':
            conversions, too_big = self.generate_struct_spec_for(msg_type)
            msg_path = self.get_msg_fullpath(msg_type)
            spec = {
                    "subscribe": [msg_path, topic],
                    "attributes": conversions,
                    }
            if too_big or getattr(msg_type, 'DONT_TRANSMIT', False):
                desc_topic = 'log_descriptions'
            else:
                desc_topic = 'sensor_descriptions'
            spec_pub = super().create_publisher(String, desc_topic, 10)
            spec_str = String()
            spec_str.data = str(spec)
            spec_pub.publish(spec_str)
        return super().create_publisher(msg_type, topic, queue)
    def create_local_publisher(self, msg_type, topic, queue=10):
        '''bypass the inheritance for special messages that we dont want to transmit'''
        return super().create_publisher(msg_type, topic, queue)
    @staticmethod
    def generate_struct_spec_for(msg_type, depth=1):
        conversions = [
                (re.compile('^byte$'), 'c'),
                (re.compile('^char$'), 'c'),
                (re.compile('^float32$'), 'f'),
                (re.compile('^float$'), 'f'),
                (re.compile('^float64$'), 'd'),
                (re.compile('^double$'), 'd'),
                (re.compile('^int8$'), 'b'),
                (re.compile('^uint8$'), 'B') ,
                (re.compile('^int16$'), 'h'),
                (re.compile('^uint16$'), 'H') ,
                (re.compile('^int32$'), 'i'),
                (re.compile('^uint32$'), 'I') ,
                (re.compile('^int64$'), 'q'),
                (re.compile('^uint64$'), 'Q') ,
                (re.compile('^string$'), 's'),
                (re.compile(r'^byte\[(\d+)\]$'), lambda m: 'c'*m),
                (re.compile(r'^char\[(\d+)\]$'), lambda m: 'c'*m),
                (re.compile(r'^float32\[(\d+)\]$'), lambda m: 'f'*m),
                (re.compile(r'^float\[(\d+)\]$'), lambda m: 'f'*m),
                (re.compile(r'^float64\[(\d+)\]$'), lambda m: 'd'*m),
                (re.compile(r'^double\[(\d+)\]$'), lambda m: 'd'*m),
                (re.compile(r'^int8\[(\d+)\]$'), lambda m: 'b'*m),
                (re.compile(r'^uint8\[(\d+)\]$'), lambda m: 'B'*m),
                (re.compile(r'^int16\[(\d+)\]$'), lambda m: 'h'*m),
                (re.compile(r'^uint16\[(\d+)\]$'), lambda m: 'H'*m),
                (re.compile(r'^int32\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile(r'^uint32\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile(r'^int64\[(\d+)\]$'), lambda m: 'q'*m),
                (re.compile(r'^uint64\[(\d+)\]$'), lambda m: 'Q'*m),
                (re.compile(r'^string\[(\d+)\]$'), lambda m: 's'*m),
                ]
        conversion_list = []
        conversion_size = 0
        too_big = False
        if depth == 0:
            return None
        for attr in msg_type.__slots__:
            for pattern, conversion in conversions:
                match = pattern.match(msg_type._fields_and_field_types[attr[1:]])
                if match:
                    if len(match.groups()) > 0:
                        n = int(match.group(1))
                        conversion_size += struct.calcsize(conversion(n))
                        if conversion_size >= Sensor.MAX_PACKET:
                            too_big = True
                        conversion_list.append([attr[1:], conversion(n)])
                    else:
                        conversion_size += struct.calcsize(conversion)
                        if conversion_size >= Sensor.MAX_PACKET:
                            too_big = True
                        conversion_list.append([attr[1:], conversion])
                    break
        return conversion_list, too_big
    @staticmethod
    def get_msg_fullpath(msg):
        msg_class = msg.__module__ + '.' + msg.__qualname__
        return msg_class

def main():
    print("this node is an abstract node. it does not run independently")

if __name__ == "__main__":
    main()
