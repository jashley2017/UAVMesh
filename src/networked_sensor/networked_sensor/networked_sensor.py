import struct
import re
import numpy as np
from std_msgs.msg import String
from environ_msgs.msg import Pth
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class Sensor(Node):
    MAX_PACKET = 70 # TODO should be configuration-based
    def __init__(self, node_name):
        super().__init__(node_name)
    def create_publisher(self, msg_type, topic, queue=10):
        '''
        Overrides the base create publisher to send a
        sensor description before the information comes back.
        The sensor is either logged or transmitted based on
        packet-size factors. The rules of which packets do
        what are as follows.
        1. If the ros msg attributes amount to a total bytesize
        larger than the maximum payload size, log only.
        2. If the ros msg type has an attribute DONT_TRANSMIT, log only.
        TODO: include approximate rate in this
        calculation if possible.
        3. If the ros msg has a dynamically sized variable do not include.
        '''
        # parameter_events is a background topic
        if topic != '/parameter_events':
            conversions, conversion_size = self.generate_struct_spec_for(msg_type)
            too_big = conversion_size >= self.MAX_PACKET
            msg_path = self.get_msg_fullpath(msg_type)
            spec = {
                "subscribe": [msg_path, topic],
                "attributes": conversions,
            }
            if too_big or getattr(msg_type, 'DONT_TRANSMIT', False): # rule 2 on docstring
                desc_topic = 'log_descriptions'
            else:
                desc_topic = 'sensor_descriptions'

            self.get_logger().info(f'asserting {spec} for {desc_topic}')
            latching_qos = QoSProfile(depth=5, # QOS profile that queues message for subscriber
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
            spec_pub = super().create_publisher(String, desc_topic, latching_qos)
            spec_str = String()
            spec_str.data = str(spec)
            spec_pub.publish(spec_str)
        return super().create_publisher(msg_type, topic, queue)
    def create_local_publisher(self, msg_type, topic, queue=10):
        '''bypass the inheritance for special messages that we dont want to transmit'''
        return super().create_publisher(msg_type, topic, queue)
    @staticmethod
    def generate_struct_spec_for(msg_type, depth=1):
        '''
        Generates a struct specification for ros msg primitives.
        TODO: be designed with a depth constrained recursion for complex messages.
        '''
        conversions = [
            (re.compile('^byte$'), 'c'),
            (re.compile('^char$'), 'c'),
            (re.compile('^float32$'), 'f'),
            (re.compile('^float$'), 'f'),
            (re.compile('^float64$'), 'd'),
            (re.compile('^double$'), 'd'),
            (re.compile('^int8$'), 'b'),
            (re.compile('^uint8$'), 'B'),
            (re.compile('^int16$'), 'h'),
            (re.compile('^uint16$'), 'H'),
            (re.compile('^int32$'), 'i'),
            (re.compile('^uint32$'), 'I'),
            (re.compile('^int64$'), 'q'),
            (re.compile('^uint64$'), 'Q'),
            # (re.compile('^string$'), 's'),
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
            # (re.compile(r'^string\[(\d+)\]$'), lambda m: 's'*m), # dynamically sized bad
        ]
        conversion_list = []
        conversion_size = 0
        if depth == 0:
            return None
        for attr in msg_type.__slots__:
            for pattern, conversion in conversions:
                match = pattern.match(msg_type._fields_and_field_types[attr[1:]])
                if match:
                    if len(match.groups()) > 0:
                        element_count = int(match.group(1))
                        conversion_size += struct.calcsize(conversion(element_count))
                        conversion_list.append([attr[1:], conversion(element_count)])
                    else:
                        conversion_size += struct.calcsize(conversion)
                        conversion_list.append([attr[1:], conversion])
                    break
        return conversion_list, conversion_size
    @staticmethod
    def get_msg_fullpath(msg):
        msg_class = msg.__module__ + '.' + msg.__qualname__
        return msg_class

def main():
    print("this node is an abstract node. it does not run independently")

if __name__ == "__main__":
    main()
