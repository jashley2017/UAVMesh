#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbee_uav',
            namespace='plane', # TODO index this variably
            executable='radio',
            name='xbee_radio')
        ])
