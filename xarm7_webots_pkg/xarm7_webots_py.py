#!/usr/bin/env python3
'''
File:   xarm7_webots_py.py
Desc:   ROS2 node interface for the xarm7 webots simulation
Date:   Aug 30 2021
Auth:   scott@restfulrobotics.com
Note:

By default, the webots node publishes /joint_states



'''

import rclpy
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_core.webots.controller import Node
from sensor_msgs.msg import JointState


class XArm7WebotsNode( WebotsNode ):

    def __init__( self, args ):
        
        super().__init__('xarmt_webots_node', args=args)

        
        self.motors = {}
        self.timestep_s = self.robot.getBasicTimeStep()/1000.0


        # Build dict of joint motors and sensors
        for idx in list(range(self.robot.getNumberOfDevices())):
            device = self.robot.getDeviceByIndex(idx)
            if device.getNodeType() in [Node.LINEAR_MOTOR, Node.ROTATIONAL_MOTOR]:
                name = device.getName()
                if device.getPositionSensor() is None:
                    self.get_logger().warn(f'Motor "{name}" doesn\'t have a position sensor...')
                else:
                    self.get_logger().info(f'Found motor and sensor for "{name}"')
                    self.motors[name] = device
                    device.getPositionSensor().enable(int(self.robot.getBasicTimeStep()))
        

        # Subsribe to enable command of robot joints
        self.create_subscription( JointState, 'target_joint_states', self.target_joint_states_callback, 10 )

    def target_joint_states_callback( self, joint_states_msg ):
        for idx,name in enumerate(joint_states_msg.name):
            self.motors[name].setPosition( joint_states_msg.position[idx] )
        
        



    
        


def main(args=None):

    rclpy.init(args=args)
    xarm7_webots_node = XArm7WebotsNode(args=args)
    xarm7_webots_node.get_logger().info('Created XArm7WebotsNode...')
    rclpy.spin( xarm7_webots_node )
    xarm7_webots_node.get_logger().info('Destroying XArm7WebotsNode...')
    xarm7_webots_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()


