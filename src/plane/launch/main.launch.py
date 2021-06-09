from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package="gps",
            namespace='plane', # TODO index this variably
            executable="neo_gps",
            name="gps",
            parameters=[
                {"gps_top": 'gps_fix'},
                {"gps_baud": 9600},
                {"gps_port": '/dev/gps'},
            ]
        ),
        Node(
            package="pth",
            namespace='plane', # TODO index this variably
            executable="pth_timeref",
            name="pth",
            parameters=[
                {"pth_top": 'pth_msg'},
                {"pth_port": '/dev/pth'},
            ]
        ),
        Node(
            package='xbee_uav',
            namespace='plane', # TODO index this variably
            executable='radio',
            name='xbee_radio',
            parameters=[
                {"xbee_port": '/dev/xbee'},
                {"xbee_baud": 9600},

            ]
        ),
        Node(
            package='plane',
            namespace='plane', # TODO: index this variably
            executable='transmitter',
            name='msg_transmitter',
            parameters=[
                {'pth_top': 'pth_msg'},
                {'gps_top': 'gps_fix'},
                {"gcu_addr": "13A20041D17945"}
            ]
        ),
    ])
    return ld
