#!/usr/bin/env python3
'''
File:   xarm7_webots_moveit.launch.py
Desc:   ROS2 launch file to start webots and a ROS2 node
Date:   Aug 30 2021
Auth:   scott@restfulrobotics.com
Note:

Example

$ cd ~/ros2_ws
$ ros2 launch src/xarm7_webots_pkg/launch/xarm7_webots_moveit.launch.py 


Additional notes

https://github.com/RIF-Robotics/ar3_ros/blob/main/ar3_moveit_config/launch/direct_joint_control.launch.py



ros2 service call controller_manager/load_and_start_controller controller_manager_msgs/srv/LoadStartController \'{name: "joint_state_controller"}\'



ros2 service call controller_manager/load_and_start_controller controller_manager_msgs/srv/LoadStartController '{name: "xarm_gripper_traj_controller"}'

To see what controllers are available

$ ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

response:
controller_manager_msgs.srv.ListControllers_Response
(
    controller=
    [
        controller_manager_msgs.msg.ControllerState(
            name='xarm_gripper_traj_controller', 
            state='active', 
            type='joint_trajectory_controller/JointTrajectoryController', 
            claimed_interfaces=
            [
                'drive_joint/position'
            ]
        ), 
        controller_manager_msgs.msg.ControllerState(
            name='xarm7_traj_controller', 
            state='active', 
            type='joint_trajectory_controller/JointTrajectoryController', 
            claimed_interfaces=
            [
                'joint1/position', 
                'joint1/velocity', 
                'joint2/position', 
                'joint2/velocity', 
                'joint3/position', 
                'joint3/velocity', 
                'joint4/position', 
                'joint4/velocity', 
                'joint5/position', 
                'joint5/velocity', 
                'joint6/position', 
                'joint6/velocity', 
                'joint7/position', 
                'joint7/velocity'
            ]
        )
    ]
)

$ ros2 service call /controller_manager/list_controller_types controller_manager_msgs/srv/ListControllerTypes 
waiting for service to become available...
requester: making request: controller_manager_msgs.srv.ListControllerTypes_Request()

response:
controller_manager_msgs.srv.ListControllerTypes_Response
(
    types=
    [
        'controller_manager/test_controller', 
        'controller_manager/test_controller_failed_init', 
        'controller_manager/test_controller_with_interfaces', 
        'diff_drive_controller/DiffDriveController', 
        'effort_controllers/GripperActionController', 
        'effort_controllers/JointGroupEffortController', 
        'forward_command_controller/ForwardCommandController', 
        'joint_state_broadcaster/JointStateBroadcaster', 
        'joint_state_controller/JointStateController', 
        'joint_trajectory_controller/JointTrajectoryController', 
        'position_controllers/GripperActionController', 
        'position_controllers/JointGroupPositionController', 
        'velocity_controllers/JointGroupVelocityController'
    ], 
    base_classes=
    [
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface', 
        'controller_interface::ControllerInterface'
    ]
)

# http://docs.ros.org/en/api/controller_manager_msgs/html/srv/SwitchController.html
$ ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
    "{ \
        start_controllers: ['position_controllers/JointGroupPositionController'], \
        stop_controllers: ['joint_trajectory_controller/JointTrajectoryController'],\
        strictness: 0, \
        start_asap: False, \
    }"



IK python code
https://leechangyo.github.io/ros/2019/10/05/ROS-TUTORIAL5/





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
    #ld.add_action(webots_node)
    ld.add_action(xarm_planner_node)
    #ld.add_action(xarm_traj_req_node)

    return ld