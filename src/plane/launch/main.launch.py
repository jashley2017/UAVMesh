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
                {"gps_port": '/dev/ttyACM0'},
            ]
        ),
        Node(
            package="pth",
            namespace='plane', # TODO index this variably
            executable="pth_timeref",
            name="pth",
            parameters=[
                {"pth_top": 'transmit'},
                {"pth_port": '/dev/ttyUSB1'},
            ]
        ),
        Node(
            package='xbee_uav',
            namespace='plane', # TODO index this variably
            executable='radio',
            name='xbee_radio')
    ])
    return ld
