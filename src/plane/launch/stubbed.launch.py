#!/usr/bin/env python
import os
from launch import LaunchDescription
from launch_ros.actions import Node 
from ament_index_python.packages import get_package_share_directory

def build_node(node_construct, arg):
    '''
    inject arguments into the node specification
    and then return the resulting node
    '''
    return node_construct[0](**node_construct[1](arg))

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('plane'),
        'config',
        'config.yaml'
    )
    defaults = {
        'radio': (Node, lambda f: {
            'package':'fake_radio',
            'namespace':'plane',
            'executable':'fake_radio',
            'name':'fake_radio',
        }),
        'gps': (Node, lambda f: {
                'package':'fake_gps',
                'namespace':'plane',
                'executable':'fake_gps',
                'name':'fake_gps',
        }),
        'sensor': (Node, lambda f: {
                'package':'fake_sensor',
                'namespace':'plane',
                'executable':'fake_sensor',
                'name':'fake_sensor',
        }),
    }
    node_list = []
    for val in defaults.values():
        node_list.append(build_node(val, None))
    node_list += [
        Node(
            package="plane",
            namespace="plane",
            executable="transmitter",
            name="msg_transmitter",
            parameters=[
                config
            ],
        ),
        Node(
            package="plane",
            namespace="plane",
            executable="logger",
            name="msg_logger",
            parameters=[
                config,
            ],
        )
    ]
    ld = LaunchDescription(node_list)
    return ld
