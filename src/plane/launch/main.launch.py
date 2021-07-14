import os
from launch_ros.actions import Node
from launch import LaunchDescription

HOSTNAME = os.uname()[1]
STR = 0
INT = 1
FLOAT = 2
HEX = 3

def generate_launch_description():
    '''create the launch description for the plane'''
    launchdesc = LaunchDescription([
        Node(
            package="gps",
            namespace=HOSTNAME,
            executable="neo_gps",
            name="gps",
            parameters=[
                {"gps_top": 'gps_fix'},
                {"gps_baud": 9600},
                {"gps_port": '/dev/gps'},
            ]
        ),
        Node(
            package="serial_timeref",
            namespace=HOSTNAME,
            executable="serial_timeref",
            parameters=[
                {'sensor_top': "pth"},
                {'sensor_msg': "environ_msgs.msg.Pth"},
                {'sensor_port': '/dev/pth'},
                {'sensor_baud': 9600},
                {'sensor_reg': r"\$UKPTH,(?P<serial>[0-9A-F]{4}),(?P<pressure>\d*\.?\d*),(?P<pressure_unit>[A-Za-z]+),(?P<temp1>\d*\.?\d*),(?P<temp1_unit>[A-Za-z]+),(?P<temp2>\d*\.?\d*),(?P<temp2_unit>[A-Za-z]+),(?P<humidity>\d*\.?\d*),(?P<hum_unit>[^0-9,]*),(?P<temp3>\d*\.?\d*),(?P<temp3_unit>[A-Za-z]+)"},
                {'time_topic': 'gps_time'},
                {'sensor_key_types':
                    { # enumerations from SerialSensor
                        "serial": HEX,
                        "pressure": FLOAT,
                        "temp1": FLOAT,
                        "temp2": FLOAT,
                        "temp3": FLOAT,
                        "humidity": FLOAT,
                    }
                }
            ]

        ),
        Node(
            package='xbee_uav',
            namespace=HOSTNAME,
            executable='radio',
            name='xbee_radio',
            parameters=[
                {"xbee_port": '/dev/xbee'},
                {"xbee_baud": 9600},

            ]
        ),
        Node(
            package='plane',
            namespace=HOSTNAME,
            executable='transmitter',
            name='msg_transmitter',
            parameters=[
                {'pth_top': 'pth_msg'},
                {'gps_top': 'gps_fix'},
                {"gcu_addr": "13A20041D17945"}
            ]
        ),
    ])
    return launchdesc
