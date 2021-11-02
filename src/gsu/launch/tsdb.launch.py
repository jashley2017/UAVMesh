#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('plane'),
        'config',
        'config.yaml'
    )
    ld = LaunchDescription([
        Node(
            package='xbee_uav',
            namespace='gsu', # TODO index this variably
            executable='radio',
            name='xbee_radio',
            parameters=[
                {"xbee_port": '/dev/xbee'},
                {"xbee_baud": 9600},
                {"is_gsu": True},

            ]
        ),
        Node(
            package="gps",
            namespace='gsu', # TODO index this variably
            executable="neo_gps",
            name="gps",
            parameters=[
                {"gps_top": 'gps_fix'},
                {"gps_baud": 9600},
                {"gps_port": '/dev/ttyACM0'},
            ]
        ),
        Node(
            package="gsu",
            namespace="gsu",
            executable="tsdb",
            name="gsu",
            parameters=[
                config
            ]
        ),
    ])
    return ld
