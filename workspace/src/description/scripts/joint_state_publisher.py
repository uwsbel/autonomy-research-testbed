#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'steering_joint',
            'lidar_joint',
            'camera_joint'
        ]
        self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []
        self.angle = 0.0

    def timer_callback(self):
        self.joint_state_msg.header = Header()
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = [
            0., #math.sin(self.angle),  # rear_left_wheel_joint
            0., #math.sin(self.angle),  # rear_right_wheel_joint
            0., #math.sin(self.angle),  # front_left_wheel_joint
            0., #math.sin(self.angle),  # front_right_wheel_joint
            0., #math.cos(self.angle) / 3,  # front_left_steering_joint
            0.,  # front_left_steering_joint
            0.,  # front_left_steering_joint
        ]
        self.angle += 0.1

        self.publisher_.publish(self.joint_state_msg)
        # self.get_logger().info('Publishing: "%s"' % str(self.joint_state_msg.position))

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
