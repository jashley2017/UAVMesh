#!/usr/bin/env python
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='uldaq_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='uldaq',
                plugin='uldaq_ros::UldaqPublisher',
                name='uldaq_publisher',
                parameters=[{
                    "v_range": 5,
                    "chan_num": 8,
                    "rate": 100,
                }]
            )
        ],
        output='screen',
        emulate_tty=True
    )

    return launch.LaunchDescription([container])
