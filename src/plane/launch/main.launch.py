#!/usr/bin/env python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import glob
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('plane'),
        'config',
        'config.yaml'
    )
    node_list = [ # required nodes
        Node(
            package="plane",
            namespace="plane",
            executable="transmitter",
            name="msg_transmitter",
            parameters=[
                config
            ],
        )
    ]
    sensor_descriptions = {
        # key: device path to match
        # value: node description to form from matched device path
        # value[0](**value[1](matched_path)) constructs the node
        '/dev/xbee*': (Node, lambda f: {
            "package":"xbee_uav",
            "namespace":"plane",
            "executable":"radio",
            "name":"xbee_radio",
            "parameters":[
                {"xbee_port": f},
                {"xbee_baud": 9600},
            ]}), # dont have more than one radio
        '/dev/gps*': (Node, lambda f: {
            "package":"gps",
            "namespace":"plane",
            "executable":"neo_gps",
            "name":"gps",
            "parameters":[
                {"gps_baud": 9600},
                {"gps_top": "gps_fix"},
                {"gps_port": f},
            ]}),
        '/dev/pth*': (Node, lambda f: {
            "package":"pth",
            "namespace":"plane",
            "executable":"pth_timeref",
            "name":"pth",
            "parameters":[
                {"pth_top": "pth_msg"},
                {"pth_port": f},
            ]}),
        '/dev/sonic*': (Node, lambda f: {
            "package":"anemometer",
            "namespace":"plane",
            "executable": "sonic_node",
            "name": "anemometer",
            "parameters": [
                {"sonic_port":f},
                {"sonic_baud": 4800},
            ]
        }),
        '/dev/daq*': (ComposableNodeContainer, lambda f: {
            "name":"uldaq_container",
            "namespace":"plane",
            "package":"rclcpp_components",
            "executable":"component_container",
            "composable_node_descriptions":[
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
            "output":"screen",
            "emulate_tty":True,
        })
    }
    for dev_path, node_construct in sensor_descriptions.items():
        for dev_path_match in glob.glob(dev_path):
            # Inject this found path into the parameters
            # and then pass the parameters to the correct
            # ROS node type.
            node_list.append(
                node_construct[0](**node_construct[1](dev_path_match))
            )
    return LaunchDescription(node_list)
