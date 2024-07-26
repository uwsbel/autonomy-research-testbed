import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from chrono_ros_interfaces.msg import Body
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import csv 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # update frequency of this node
        self.freq = 10.0

        # initialize control inputs
        self.steering_leader = 0.0
        self.throttle_leader = 0.7
        
        self.steering_follower = 0.0
        self.throttle_follower = 0.3
        
        self.braking = 0.0

        # initialize vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0

        # data that will be used by this class
        self.vehicle_states = []
        self.path = Path()
        self.go = False
        self.vehicle_cmd = VehicleInput()
        self.odometry_buffer = []
        self.odometry_buffer_size = 50
        self.convergence_threshold = 0.01

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub_harryInput = self.create_subscription(Twist, '/leader_vehicle/cmd_vel', self.HarryInputs_callback, qos_profile)
        self.sub_odometry = self.create_subscription(Odometry, '/leader_vehicle/odometry/filtered', self.odometry_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '/leader_vehicle/control/vehicle_inputs', 10)
        self.pub_vehicle_state = self.create_publisher(Path, '/leader_vehicle/vehicle_traj', 10)
        
        # Service to start publishing path
        self.srv_start_publishing = self.create_service(Trigger, 'start_publishing_path', self.start_publishing_callback)
        self.path_publishing_enabled = False
        
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

    # subscribe manual control inputs
    def HarryInputs_callback(self, msg):
        #self.go = True
        #self.get_logger().info("received harry's inputs:")
        self.throttle_leader += msg.linear.x
        self.steering_leader += msg.angular.z
        self.get_logger().info("Throttle: %s" % self.throttle_leader)
        self.get_logger().info("Steering: %s" % self.steering_leader)

    # new callback for odometry messages
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #convert quaternion to euler angles
        e0 = msg.pose.pose.orientation.x
        e1 = msg.pose.pose.orientation.y
        e2 = msg.pose.pose.orientation.z
        e3 = msg.pose.pose.orientation.w
        self.theta = np.arctan2(2.0 * (e3 * e2 + e0 * e1), 1.0 - 2.0 * (e1 * e1 + e2 * e2))        
        self.v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)

        # Save odometry to buffer for convergence check
        self.odometry_buffer.append((self.x, self.y))
        if len(self.odometry_buffer) > self.odometry_buffer_size:
            self.odometry_buffer.pop(0)
        
        self.go = True

    def start_publishing_callback(self, request, response):
        self.path_publishing_enabled = True
        response.success = True
        response.message = "Started publishing vehicle path."
        return response
        
    def vehicle_trajectory(self):
        if not self.go:
            return self.path

        if not self.path_publishing_enabled:
            return self.path

        if len(self.path.poses) > 0:
            last_pose = self.path.poses[-1].pose.position
            distance = np.sqrt((self.x - last_pose.x) ** 2 + (self.y - last_pose.y) ** 2)
            if distance < 1.0:
                return self.path

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.z = np.sin(self.theta / 2)
        pose.pose.orientation.w = np.cos(self.theta / 2)

        self.path.poses.append(pose)

        if len(self.path.poses) > 30:
            self.path.poses.pop(0)

        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "map"
        return self.path
        
    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        state_array = self.vehicle_trajectory()
        ### for vehicle one
        msg_leader = VehicleInput()
        msg_leader.steering = np.clip(self.steering_leader, -1.0, 1.0)
        msg_leader.throttle = np.clip(self.throttle_leader, 0, 1)
        
        if self.path_publishing_enabled:
            self.pub_vehicle_state.publish(state_array)

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control, executor=executor)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
