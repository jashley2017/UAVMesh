#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
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
            executable="logger",
            name="gsu",
            parameters=[
                {'time_topic': "gps_time"},
                {'outfile': "/home/ubuntu/outfile.log"},
            ]
        ),
    ])
    return ld
