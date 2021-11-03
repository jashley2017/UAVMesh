import re
from sensor_msgs.msg import NavSatFix

def parse_msg(msg_type, depth=1):
    conversions = [
            (re.compile('^byte$'), ('c', 1)),
            (re.compile('^char$'), ('c', 1)),
            (re.compile('^float32$'), ('f', 4)),
            (re.compile('^float64$'), ('d', 8)),
            (re.compile('^double$'), ('d', 8)),
            (re.compile('^int8$'), ('i', 1)),
            (re.compile('^uint8$'), ('I', 1)) ,
            (re.compile('^int8$'), ('i', 1)),
            (re.compile('^uint8$'), ('I', 1)) ,
            (re.compile('^int16$'), ('i', 2)),
            (re.compile('^uint16$'), ('I', 2)) ,
            (re.compile('^int32$'), ('i', 4)),
            (re.compile('^uint32$'), ('I', 4)) ,
            (re.compile('^int64$'), ('i', 8)),
            (re.compile('^uint64$'), ('I', 8)) ,
            (re.compile('^string$'), ('s', None)),
            (re.compile('^byte\[(\d+)\]$'), lambda m: ('c'*m, 1*m)),
            (re.compile('^char\[(\d+)\]$'), lambda m: ('c'*m, 1*m)),
            (re.compile('^float32\[(\d+)\]$'), lambda m: ('f'*m, 4*m)),
            (re.compile('^float64\[(\d+)\]$'), lambda m: ('d'*m, 8*m)),
            (re.compile('^double\[(\d+)\]$'), lambda m: ('d'*m, 8*m)),
            (re.compile('^int8\[(\d+)\]$'), lambda m: ('i'*m, 1*m)),
            (re.compile('^uint8\[(\d+)\]$'), lambda m: ('I'*m, 1*m)),
            (re.compile('^int8\[(\d+)\]$'), lambda m: ('i'*m, 1*m)),
            (re.compile('^uint8\[(\d+)\]$'), lambda m: ('I'*m, 1*m)),
            (re.compile('^int16\[(\d+)\]$'), lambda m: ('i'*m, 2*m)),
            (re.compile('^uint16\[(\d+)\]$'), lambda m: ('I'*m, 2*m)),
            (re.compile('^int32\[(\d+)\]$'), lambda m: ('i'*m, 4*m)),
            (re.compile('^uint32\[(\d+)\]$'), lambda m: ('I'*m, 4*m)),
            (re.compile('^int64\[(\d+)\]$'), lambda m: ('i'*m, 8*m)),
            (re.compile('^uint64\[(\d+)\]$'), lambda m: ('I'*m, 8*m)),
            (re.compile('^string\[(\d+)\]$'), lambda m: ('s'*m, None*m)),
            ]
    conversion_list = [[], []] 
    for attr in msg_type.__slots__:
        for pattern, conversion in conversions:
            match = pattern.match(msg_type._fields_and_field_types[attr[1:]])
            if match: 
                conversion_list[0].append(attr[1:])
                if len(match.groups()) > 0:
                    n = int(match.group(1))
                    conversion_list[1].append(conversion(n))
                else:
                    conversion_list[1].append(conversion) 
                break
    print(conversion_list)

if __name__ == '__main__':
    parse_msg(NavSatFix)
