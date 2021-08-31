#!/usr/bin/env python3
'''
File:   xarm7_webots_moveit.launch.py
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

    #static frame, world_tx_base_link
    static_frame_world_tx_link_base = Node(
        package= 'tf2_ros', 
        executable= 'static_transform_publisher',
        output='screen',
        arguments = ['0', '0', '0', '0', '0', '0', 'world', 'link_base']
    )

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
            ('publish_tf', 'False') #Note the xarm planner publishes the robot state
        ]
    ) #webots_node

    # Call xarm7 launch file
    xarm_planner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_planner'), 'launch', 'xarm7_planner_fake.launch.py' )
        ),
        launch_arguments=[
            ('add_gripper', 'True')
        ]
    )

    # Traj. requests
    xarm_traj_req_node = Node(
        package='xarm_ros2_client_py_pkg',
        executable='xarm_ros2_client_py',
        output='screen',
    )


    ld.add_action(static_frame_world_tx_link_base)
    ld.add_action(webots_node)
    ld.add_action(xarm_planner_node)
    ld.add_action(xarm_traj_req_node)

    return ld