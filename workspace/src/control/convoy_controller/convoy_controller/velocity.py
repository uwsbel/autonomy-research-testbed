import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        # Initialize parameters
        self.kp = self.declare_parameter('velocity_kp', 0.5).value
        self.ki = self.declare_parameter('velocity_ki', 0.1).value
        self.kd = self.declare_parameter('velocity_kd', 0.1).value
        self.error_integral_limit = self.declare_parameter('velocity_error_integral_limit', 5.0).value
        self.target_velocity = self.declare_parameter('target_velocity', 0.2).value

        self.prev_error = 0
        self.error_sum = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )


        # Subscribers and publishers
        self.sub_odometry = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.pub_throttle = self.create_publisher(VehicleInput, '/driver_inputs', qos_profile)
        
        # Initialize vehicle state
        self.v = 0.0

        self.timer = self.create_timer(0.1, self.pub_callback)

    def odometry_callback(self, msg):
        self.v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)

    def compute_throttle(self, current_velocity, dt):
        error = self.target_velocity - current_velocity
        error_rate = (error - self.prev_error) / dt
        self.prev_error = error

        self.error_sum += error * dt
        if self.error_sum > self.error_integral_limit:
            self.error_sum = self.error_integral_limit
        elif self.error_sum < -self.error_integral_limit:
            self.error_sum = -self.error_integral_limit

        throttle = self.kp * error + self.ki * self.error_sum + self.kd * error_rate
        return np.clip(throttle, 0, 1)

    def pub_callback(self):
        dt = 0.1  # Assuming a fixed dt for simplicity
        throttle = self.compute_throttle(self.v, dt)
        
        msg = VehicleInput()
        msg.throttle = throttle
        self.pub_throttle.publish(msg)

        # self.get_logger().info(f'Published {throttle}.')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
