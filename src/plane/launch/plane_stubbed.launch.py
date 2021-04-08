from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    ld = LaunchDescription([
        Node(
            package="gps",
            namespace='plane', # TODO index this variably
            executable="gps_node",
            name="gps",
            parameters=[
                {"gps_top": 'transmit'},
            ]
        ),
        Node(
            package="pth",
            namespace='plane', # TODO index this variably
            executable="pth_node",
            name="pth",
            parameters=[
                {"pth_top": 'transmit'},
            ]
        ),
        Node(
            package='xbee_uav',
            namespace='plane', # TODO index this variably
            executable='radio',
            name='xbee_radio')
    ])
    return ld
