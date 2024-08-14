#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Path, Odometry
import numpy as np
from math import atan2, cos, sin, sqrt

class PathGeneratorNode(Node):
    def __init__(self):
        super().__init__('path_generator_node')
        self.initial_pose_received = False
        self.initial_pose = None

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10)

        self.path_publisher = self.create_publisher(Path, '/path', 10)

        # Set the spacing between points
        self.spacing = 0.2  # Adjust this value to change the spacing between points in meters

    def odom_callback(self, msg):
        self.initial_pose = msg.pose.pose
        self.initial_pose_received = True

    def goal_pose_callback(self, msg):
        if not self.initial_pose_received:
            self.get_logger().warn('Initial pose not received yet. Waiting for initial pose...')
            return

        goal_pose = msg.pose
        self.get_logger().info(f'Received goal pose: {goal_pose}')
        
        # Define angle constraints (in radians) and magnitudes for tangents
        start_angle = self.get_yaw_from_quaternion(self.initial_pose.orientation)
        start_magnitude = 5.0  # Adjust this value to change the magnitude of the start tangent
        goal_angle = self.get_yaw_from_quaternion(goal_pose.orientation)
        goal_magnitude = 5.0  # Adjust this value to change the magnitude of the goal tangent
        
        # Generate Bezier spline trajectory
        trajectory = self.generate_bezier_spline_trajectory(self.initial_pose, goal_pose, start_angle, start_magnitude, goal_angle, goal_magnitude)

        # Publish the trajectory
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for pose in trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose = pose
            path_msg.poses.append(pose_stamped)
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info('Published Bezier spline trajectory')

    def generate_bezier_spline_trajectory(self, initial_pose, goal_pose, start_angle, start_magnitude, goal_angle, goal_magnitude):
        start_x = initial_pose.position.x
        start_y = initial_pose.position.y

        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        # Control points
        p0 = np.array([start_x, start_y])
        p1 = p0 + np.array([cos(start_angle), sin(start_angle)]) * start_magnitude
        p2 = np.array([goal_x, goal_y]) - np.array([cos(goal_angle), sin(goal_angle)]) * goal_magnitude
        p3 = np.array([goal_x, goal_y])

        trajectory = []
        t = 0.0
        prev_point = p0

        while t < 1.0:
            # Increase t incrementally until the desired spacing is achieved
            t_next = t
            while t_next < 1.0:
                t_next += 0.001  # Small step to increment t
                point = (1 - t_next)**3 * p0 + 3 * (1 - t_next)**2 * t_next * p1 + 3 * (1 - t_next) * t_next**2 * p2 + t_next**3 * p3
                distance = sqrt((point[0] - prev_point[0])**2 + (point[1] - prev_point[1])**2)
                if distance >= self.spacing:
                    break

            if t_next >= 1.0:
                break

            t = t_next
            prev_point = point

            tangent = 3 * (1 - t)**2 * (p1 - p0) + 6 * (1 - t) * t * (p2 - p1) + 3 * t**2 * (p3 - p2)
            theta = atan2(tangent[1], tangent[0])
            pose = Pose()
            pose.position.x = point[0]
            pose.position.y = point[1]
            pose.orientation = self.get_quaternion_from_yaw(theta)
            trajectory.append(pose)

        # Ensure the last point is exactly the goal pose
        pose = Pose()
        pose.position.x = p3[0]
        pose.position.y = p3[1]
        pose.orientation = goal_pose.orientation
        trajectory.append(pose)

        return trajectory

    def get_yaw_from_quaternion(self, quat):
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return atan2(siny_cosp, cosy_cosp)

    def get_quaternion_from_yaw(self, yaw):
        # Convert yaw to quaternion
        quat = Pose().orientation
        quat.w = cos(yaw / 2.0)
        quat.z = sin(yaw / 2.0)
        quat.x = 0.0
        quat.y = 0.0
        return quat

def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
