import struct
import time
import re
from std_msgs.msg import String

class Sensor(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
    def create_publisher(self, msg_type, topic, queue=10):
        '''overrides the base create publisher to send a sensor description before the information comes back'''
        spec_pub = super().create_publisher(String, 'sensor_descriptions', 10)
        conversions = self.parse_msg(msg_type)
        msg_path = self.get_msg_fullpath(msg_type)
        spec = {"subscribe": [msg_path, topic],
                "attributes": conversions}
        spec_str = String()
        spec_str.data = str(spec)
        spec_pub.publish(spec_str)
        return super().create_publisher(msg_type, topic, queue)
    def create_local_publisher(self, msg_type, topic, queue=10):
        '''bypass the inheritance for special messages that we dont want to transmit'''
        return super().create_publisher(msg_type, topic, queue)
    @staticmethod
    def parse_msg(msg_type, depth=1):
        conversions = [
                (re.compile('^byte$'), 'c'),
                (re.compile('^char$'), 'c'),
                (re.compile('^float32$'), 'f'),
                (re.compile('^float64$'), 'd'),
                (re.compile('^double$'), 'd'),
                (re.compile('^int8$'), 'i'),
                (re.compile('^uint8$'), 'I') ,
                (re.compile('^int8$'), 'i'),
                (re.compile('^uint8$'), 'I') ,
                (re.compile('^int16$'), 'i'),
                (re.compile('^uint16$'), 'I') ,
                (re.compile('^int32$'), 'i'),
                (re.compile('^uint32$'), 'I') ,
                (re.compile('^int64$'), 'i'),
                (re.compile('^uint64$'), 'I') ,
                (re.compile('^string$'), 's'),
                (re.compile('^byte\[(\d+)\]$'), lambda m: 'c'*m),
                (re.compile('^char\[(\d+)\]$'), lambda m: 'c'*m),
                (re.compile('^float32\[(\d+)\]$'), lambda m: 'f'*m),
                (re.compile('^float64\[(\d+)\]$'), lambda m: 'd'*m),
                (re.compile('^double\[(\d+)\]$'), lambda m: 'd'*m),
                (re.compile('^int8\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile('^uint8\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile('^int8\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile('^uint8\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile('^int16\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile('^uint16\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile('^int32\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile('^uint32\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile('^int64\[(\d+)\]$'), lambda m: 'i'*m),
                (re.compile('^uint64\[(\d+)\]$'), lambda m: 'I'*m),
                (re.compile('^string\[(\d+)\]$'), lambda m: 's'*m),
                ]
        conversion_list = []
        for attr in msg_type.__slots__:
            for pattern, conversion in conversions:
                match = pattern.match(msg_type._fields_and_field_types[attr[1:]])
                if match:
                    if len(match.groups()) > 0:
                        n = int(match.group(1))
                        conversion_list.append([attr[1:], conversion(n)])
                    else:
                        conversion_list.append([attr[1:], conversion])
                    break
        return conversion_list
    @staticmethod
    def get_msg_fullpath(msg):
        msg_class = msg.__module__ + '.' + msg.__qualname__
        return msg_class

if __name__ == "__main__":
    print("this node is an abstract node. it does not run independently")

''' TODO:
this is here as a reference of the original intention for how the specification works once
it is functional this should be removed.


codes = {"0A23A1F2": [(0,0),('gps', "sensor_msgs.msg.NavSatFix")] }
specs = {}

def rebuild_data(msg):
    code = int(msg.data[0])
    msg_stamp = _unpack_bytelist(msg.data[1:9], vartype='d')
    ts = time.time()
    roundtrip_time = ts - msg_stamp

    msg_type = codes[msg.dev_addr][code][1]
    topic = codes[msg.dev_addr][code][0]
    spec = specs[msg_type]

    fields = {}
    tags = {"PlaneID": msg.dev_addr}
    byte_index = 9
    import pdb;pdb.set_trace()
    for field, field_type in spec:
        fields[field] = _unpack_bytelist(msg.data[byte_index: byte_index+struct.calcsize(field_type)], vartype=field_type)
        byte_index += struct.calcsize(field_type)
    fields["timelag"] = roundtrip_time
    samples =  [{
            "measurement": topic,
            "tags": tags,
            "time": int(msg_stamp*1000),
            "fields": fields
    }]
    return samples

class Object(object):
    pass

if __name__ == '__main__':
    desc = {"subscribe": [Sensor.get_msg_fullpath(NavSatFix), 'gps_msg'], "attributes": Sensor.parse_msg(NavSatFix)}
    import pdb;pdb.set_trace()
    specs[Sensor.get_msg_fullpath(NavSatFix)] = desc['attributes']
    desc_str = String()
    desc_str.data = str(desc)
    gps_callback = _generate_callback(desc_str)
    stub_msg = NavSatFix()
    stub_msg.longitude = 1.
    stub_msg.latitude = 2.
    stub_msg.altitude = 3.
    stub_msg.position_covariance = [1.,1., 1., 1., 1., 1., 1., 1.,1.]
    stub_msg.position_covariance_type = 0
    bytelist = gps_callback(stub_msg)
    msg = Object()
    setattr(msg, 'data', bytelist)
    setattr(msg, 'dev_addr', "0A23A1F2")
    print(rebuild_data(msg))
'''
