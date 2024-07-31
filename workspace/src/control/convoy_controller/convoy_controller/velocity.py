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

        self.leader_ns = self.declare_parameter('leader_ns', 'none').value

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
        self.sub_follower_odometry = self.create_subscription(Odometry, '/follower/odometry/filtered', self.follower_odometry_callback, 10)
        self.sub_leader_odometry = self.create_subscription(Odometry, '/leader/odometry/filtered', self.leader_odometry_callback, 10)
        self.pub_throttle = self.create_publisher(VehicleInput, '/driver_inputs', qos_profile)
        
        # Initialize vehicle state
        self.follower_velocity = 0.0
        self.leader_position = None
        self.follower_position = None

        self.freq = 10.0

        self.timer = self.create_timer(1.0 / self.freq, self.pub_callback)

        self.get_logger().info('##### Longitudinal Controller initialized with the following parameters:')
        self.get_logger().info(f' - Frequency: {self.freq} Hz')
        self.get_logger().info(f' - Velocity KP: {self.kp}')
        self.get_logger().info(f' - Velocity KI: {self.ki}')
        self.get_logger().info(f' - Velocity KD: {self.kd}')
        self.get_logger().info(f' - Target Velocity: {self.target_velocity}')
        self.get_logger().info(f' - Velocity CTE Integral Limit: {self.error_integral_limit}')

    def follower_odometry_callback(self, msg):
        self.follower_velocity = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
        self.follower_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def leader_odometry_callback(self, msg):
        self.leader_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

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
        brake = 0.0
        if self.leader_ns != 'none' and self.leader_position is not None and self.follower_position is not None:
            distance = np.linalg.norm(self.leader_position - self.follower_position)
            if distance < 1.0:
                throttle = 0.0
                brake = 0.8
                self.get_logger().info('##### Brakes Cut')

            else:
                dt = 1.0 / self.freq  # Using the timer frequency to calculate dt
                throttle = self.compute_throttle(self.follower_velocity, dt)
        else:
            dt = 1.0 / self.freq  # Using the timer frequency to calculate dt
            throttle = self.compute_throttle(self.follower_velocity, dt)

        msg = VehicleInput()
        msg.throttle = throttle
        msg.braking = brake
        self.pub_throttle.publish(msg)

        # self.get_logger().info(f'Published throttle: {throttle}.')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
