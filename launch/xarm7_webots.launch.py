#!/usr/bin/env python3
'''
File:   xarm7_webots.launch.py
Desc:   ROS2 launch file to start webots and a ROS2 node
Date:   Aug 30 2021
Auth:   scott@restfulrobotics.com
Note:


'''

import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    #webots node
    webots_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'xarm7_webots_pkg'),
            ('executable', 'xarm7_webots_node'),
            ('output', 'screen'),
            ('world', '/home/snortman/ros2_ws/src/xarm7_webots_pkg/webots/worlds/xarm7.wbt'),
            ('node_prefix', 'webots'),
            ('publish_tf', 'False') # stops robot tf publisher and joint state publisher
        ]
    ) #webots_node

    ld.add_action(webots_node)

    return ld




