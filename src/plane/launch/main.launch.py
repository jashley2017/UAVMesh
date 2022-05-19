#!/usr/bin/env python
import glob
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
    node_list = [] # nodes to be launched
    radio_descriptions = {
        '/dev/xbee*': (Node, lambda f: {
            "package":"xbee_uav",
            "namespace":"plane",
            "executable":"radio",
            "name":"xbee_radio",
            "parameters":[
                {"xbee_port": f},
                {"xbee_baud": 9600},
            ]}), # dont have more than one radio
    }
    gps_descriptions = {
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
    }
    sensor_descriptions = {
        # key: device path to match
        # value: node description to form from matched device path
        # value[0](**value[1](matched_path)) constructs the node
        '/dev/pth*': (Node, lambda f: {
            "package":"pth",
            "namespace":"plane",
            "executable":"pth_timeref",
            "name":"pth",
            "parameters":[
                {"pth_top": "pth_msg"},
                {"pth_port": f},
            ]
        }),
        '/dev/imet*': (Node, lambda f: {
            "package": "iMET",
            "namespace": "plane",
            "executable": "imet_transmitter",
            "name": "iMET_topic",
            "parameters": [
            {"iMET_port":f},
            {"iMET_baud": 57600},
            ]
        }),
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
    }
    # turn on sensors
    for dev_path, node_construct in sensor_descriptions.items():
        for dev_path_match in glob.glob(dev_path):
            node_list.append(build_node(node_construct, dev_path_match))
    # turn on radio
    has_radio = False
    for dev_path, node_construct in radio_descriptions.items():
        for dev_path_match in glob.glob(dev_path):
            has_radio = True
            node_list.append(build_node(node_construct, dev_path_match))
    if not has_radio:
        node_list.append(build_node(defaults['radio'], None))
    # turn on gps
    has_gps = False
    for dev_path, node_construct in gps_descriptions.items():
        for dev_path_match in glob.glob(dev_path):
            has_gps = True
            node_list.append(build_node(node_construct, dev_path_match))
    if not has_gps:
        node_list.append(build_node(defaults['gps'], None))
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
                {'logall': not has_radio} # if no radio, log everything
            ],
        )
    ]
    return LaunchDescription(node_list)
