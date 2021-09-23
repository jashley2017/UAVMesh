from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription(
        [
            Node(
                package="gps",
                namespace="plane",
                executable="neo_gps",
                name="gps",
                parameters=[
                    {"gps_top": "gps_fix"},
                    {"gps_baud": 9600},
                    {"gps_port": "/dev/gps"},
                ],
            ),
            Node(
                package="pth",
                namespace="plane",
                executable="pth_timeref",
                name="pth",
                parameters=[
                    {"pth_top": "pth_msg"},
                    {"pth_port": "/dev/pth"},
                ],
            ),
            Node(
                package="xbee_uav",
                namespace="plane",
                executable="radio",
                name="xbee_radio",
                parameters=[
                    {"xbee_port": "/dev/xbee"},
                    {"xbee_baud": 9600},
                ],
            ),
            ComposableNodeContainer(
                name="uldaq_container",
                namespace="plane",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="uldaq",
                        plugin="uldaq_ros::UldaqPublisher",
                        name="uldaq_publisher",
                        parameters=[
                            {
                                "v_range": 5,
                                "chan_num": 8,
                                "rate": 1000,
                            }
                        ],
                    )
                ],
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="plane",
                namespace="plane",
                executable="transmitter",
                name="msg_transmitter",
                parameters=[
                    {"pth_top": "pth_msg"},
                    {"gps_top": "gps_fix"},
                    {"gcu_addr": "13A20041D17945"},
                ],
            ),
        ]
    )
    return ld
