import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
# from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from art_msgs.msg import VehicleInput

from chrono_ros_interfaces.msg import Body
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

import csv 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger


class SteeringController:
    def __init__(self, node):
        self.node = node
        self.kp = node.declare_parameter('steering_kp', 1.0).value
        self.ki = node.declare_parameter('steering_ki', 0.0).value
        self.kd = node.declare_parameter('steering_kd', -0.02).value
        self.cte_integral_limit = node.declare_parameter('steering_cte_integral_limit', 5.0).value

        self.prev_cte = 0
        self.cte_sum = 0
        self.lookahead = 2.0
        self.steering = 0.0

    def compute_steering(self, x, y, theta, ref_traj, v, dt):
        # Calculate error state
        dist = np.zeros((1, len(ref_traj)))
        for i in range(len(ref_traj)):
            dist[0][i] = (x + np.cos(theta) * self.lookahead - ref_traj[i][0]) ** 2 + (y + np.sin(theta) * self.lookahead - ref_traj[i][1]) ** 2
        index = dist.argmin()

        ref_state_current = ref_traj[index]
        
        dx = ref_state_current[0] - x
        dy = ref_state_current[1] - y
        ref_theta = np.arctan2(dy, dx)

        # Compute the angular error
        err_theta = ref_theta - theta

        # Normalize the angular error to be within [-pi, pi]
        while err_theta < -np.pi:
            err_theta += 2 * np.pi
        while err_theta > np.pi:
            err_theta -= 2 * np.pi

        # Compute cross-track error (CTE)
        cte = np.sin(err_theta) * np.sqrt(dx**2 + dy**2)
        
        # Compute cross-track error rate (CTE rate)
        cte_rate = (cte - self.prev_cte) / dt
        self.prev_cte = cte

        # CTE Integral term
        self.cte_sum += cte * dt
        if self.cte_sum > self.cte_integral_limit:
            self.cte_sum = self.cte_integral_limit
        elif self.cte_sum < -self.cte_integral_limit:
            self.cte_sum = -self.cte_integral_limit

        # Calculate steering
        L = 0.02
        steering = ((self.kp * cte) + (self.ki * self.cte_sum) + (self.kd * cte_rate))*(L/((v+0.01)))


        # ensure steering can't change too much between timesteps, smooth transition
        delta_steering = steering - self.steering
        if abs(delta_steering) > 0.1:
            self.steering = self.steering + 0.1 * delta_steering / abs(delta_steering)
        else:
            self.steering = steering
        
        return np.clip(steering, -1.0, 1.0), [self.kp * cte*1/((v+0.01)), self.ki*self.cte_sum*1/((v+0.01)), self.kd * cte_rate*1/((v+0.01))]


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Update frequency of this node
        self.freq = 10.0

        # Initialize control inputs
        self.steering = 0.0
        self.throttle = 0.7
        self.braking = 0.0

        # Initialize vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0

        # Leader vehicle state
        self.leader_x = 0.0
        self.leader_y = 0.0

        # Data that will be used by this class
        self.vehicle_states = []
        self.path = Path()
        self.go = False
        self.vehicle_cmd = VehicleInput()
        
        self.ref_traj = []
        
        # Parameter to determine if using predefined path
        self.predefined_path = self.declare_parameter('predefined_path', True).value

        # Initialize controllers
        self.steering_controller = SteeringController(self)

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=10)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub_odometry = self.create_subscription(Odometry, '/follower_vehicle/odometry/filtered', self.odometry_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '/follower_vehicle/driver_inputs', qos_profile)

        if self.predefined_path:
            self.sub_vehicle_state = self.create_subscription(Path, '/path', self.trajectory_callback, qos_profile)
        else:
            self.sub_vehicle_state = self.create_subscription(Path, '/leader_vehicle/vehicle_traj', self.trajectory_callback, qos_profile)
            self.sub_leader_odometry = self.create_subscription(Odometry, '/leader_vehicle/odometry/filtered', self.leader_odometry_callback, qos_profile)

        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        
        # Service to start the node
        self.srv = self.create_service(Trigger, 'start_control', self.start_control_callback)

        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('##### 1 Controller initialized with the following parameters:')
        self.get_logger().info(f' - Frequency: {self.freq} Hz')
        self.get_logger().info(f' - Predefined Path: {self.predefined_path}')
        self.get_logger().info(f' - Steering KP: {self.steering_controller.kp}')
        self.get_logger().info(f' - Steering KI: {self.steering_controller.ki}')
        self.get_logger().info(f' - Steering KD: {self.steering_controller.kd}')
        self.get_logger().info(f' - Steering CTE Integral Limit: {self.steering_controller.cte_integral_limit}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'steering_kp':
                self.steering_controller.kp = param.value
            elif param.name == 'steering_ki':
                self.steering_controller.ki = param.value
            elif param.name == 'steering_kd':
                self.steering_controller.kd = param.value
            elif param.name == 'steering_cte_integral_limit':
                self.steering_controller.cte_integral_limit = param.value
        return rclpy.parameter.Parameter.Type.PARAMETER_CHANGED

    def trajectory_callback(self, msg):
        self.go = True
        self.ref_traj = [(pose.pose.position.x, pose.pose.position.y, 
                            np.arctan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y), 
                             1.0 - 2.0 * (pose.pose.orientation.y ** 2 + pose.pose.orientation.z ** 2)), 
                            0) for pose in msg.poses]

        self.get_logger().info(f'Received path with {len(self.ref_traj)} points.')
        
    # Function to process data this class subscribes to
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convert quaternion to Euler angles
        e0 = msg.pose.pose.orientation.x
        e1 = msg.pose.pose.orientation.y
        e2 = msg.pose.pose.orientation.z
        e3 = msg.pose.pose.orientation.w

        self.theta = np.arctan2(2.0 * (e3 * e2 + e0 * e1), 1.0 - 2.0 * (e1 * e1 + e2 * e2))
        #self.theta = np.arctan2(2 * (e0 * e3 + e1 * e2), e0**2 + e1**2 - e2**2 - e3**2)

        self.v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
    
    def leader_odometry_callback(self, msg):
        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y

    # Callback to run a loop and publish data this class generates
    def pub_callback(self):
        if not self.go:
            return
        
        dt = 1 / self.freq

        self.steering, error_state = self.steering_controller.compute_steering(self.x, self.y, self.theta, self.ref_traj, self.v, dt)
        
        self.get_logger().info(f"ERROR STATE {error_state}")
        # if not self.predefined_path:
        #     distance_to_leader = self.distance((self.x, self.y), (self.leader_x, self.leader_y))
        #     if distance_to_leader <= 3.0:
        #         self.throttle = 0.0
        #         self.get_logger().info("Within 3 meter of the leader vehicle, cutting throttle")
        #     else:
        #         self.throttle = 0.7  # Default throttle value, you might want to update this logic
        # else:
        #     self.throttle = 0.7  # Default throttle value, you might want to update this logic
        
        ### For vehicle one
        msg_follower = VehicleInput()
        msg_follower.steering = self.steering
        msg_follower.throttle = 0.0
        
        self.pub_vehicle_cmd.publish(msg_follower)
        
        self.get_logger().info(f"## Control Inputs {self.steering} {self.throttle}")
        self.get_logger().info(f"## Velocity {self.v} ")

    
    def start_control_callback(self, request, response):
        self.go = True
        response.success = True
        response.message = "Control node started"
        self.get_logger().info("Received start signal")
        return response

    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control, executor)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
