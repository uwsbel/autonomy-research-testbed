#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import curses
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from std_srvs.srv import SetBool
import math
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        qos_profile = QoSProfile(depth=10)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.publisher_ = self.create_publisher(VehicleInput, '/artcar_1/input/lateral_input', qos_profile)
        self.subscription_imu = self.create_subscription(Imu, '/artcar_1/imu/data', self.imu_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/artcar_1/odometry/filtered', self.odom_callback, 10)
        self.service_client = self.create_client(SetBool, '/artcar_1/start_publishing_path')  # Service client

        self.steering = 0.0
        self.throttle = 0.0
        self.speed_increment = 0.1
        self.steer_increment = 0.1
        self.imu_heading = 0.0
        self.odom_heading = 0.0
        self.velocity = 0.0  # Initialize velocity variable

    def run(self):
        curses.wrapper(self.curses_loop)

    def curses_loop(self, stdscr):
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "ART Keyboard Teleop")
        stdscr.addstr(1, 0, "Use arrow keys to control the robot")
        stdscr.addstr(2, 0, "UP/DOWN: Throttle, LEFT/RIGHT: Steering")
        stdscr.addstr(3, 0, "X: Zero out inputs")
        stdscr.addstr(4, 0, "S: Start Publishing Path")
        stdscr.addstr(5, 0, "Q: Quit")

        while rclpy.ok():
            key = stdscr.getch()

            if key == curses.KEY_UP:
                self.throttle = min(1.0, self.throttle + self.speed_increment)
            elif key == curses.KEY_DOWN:
                self.throttle = max(0.0, self.throttle - self.speed_increment)
            elif key == curses.KEY_RIGHT:
                self.steering = max(-1.0, self.steering - self.steer_increment)
            elif key == curses.KEY_LEFT:
                self.steering = min(1.0, self.steering + self.steer_increment)
            elif key == ord('x'):
                self.steering = 0.0
                self.throttle = 0.0
            elif key == ord('s'):
                self.call_start_publishing_service(True)  # Start publishing path
            elif key == ord('q'):
                break

            self.publish_vehicle_input()
            stdscr.addstr(7, 0, f"Throttle: {self.throttle:.2f}   Steering: {self.steering:.2f}")
            stdscr.addstr(8, 0, f"IMU Heading: {self.imu_heading:.2f}")
            stdscr.addstr(9, 0, f"Odometry Heading: {self.odom_heading:.2f}")
            stdscr.addstr(10, 0, f"Velocity: {self.velocity:.2f} m/s")  # Display velocity
            stdscr.refresh()
            rclpy.spin_once(self)

    def publish_vehicle_input(self):
        msg = VehicleInput()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.steering = self.steering
        msg.throttle = self.throttle
        self.publisher_.publish(msg)

    def call_start_publishing_service(self, start):
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        request = SetBool.Request()
        request.data = start
        future = self.service_client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call success: {response.success}, message: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def imu_callback(self, msg):
        # Convert quaternion to yaw angle
        orientation_q = msg.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.imu_heading = math.degrees(yaw)

    def odom_callback(self, msg):
        # Convert quaternion to yaw angle
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_heading = math.degrees(yaw)
        if self.odom_heading < 0:
            self.odom_heading += 360

        # Extract linear velocity
        linear_velocity = msg.twist.twist.linear
        self.velocity = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)  # Calculate magnitude of velocity

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    teleop_node.run()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

