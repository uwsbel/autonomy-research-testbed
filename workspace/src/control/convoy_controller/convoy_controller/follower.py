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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

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
        self.lookahead = 1.0
        
        # Initialize path tracking attributes
        self.odom_path = []
        self.path_pub_interval = 1.0  # seconds
        self.path_last_pub_time = self.get_clock().now().to_msg()
        self.path_length_limit = 30
        self.path_pub_distance_threshold = 1.0  # meters

        # Flag to start publishing odometry path
        self.start_publishing_path = False

        # Parameter to determine if using predefined path
        self.predefined_path = self.declare_parameter('predefined_path', True).value

        # Publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub_odometry = self.create_subscription(Odometry, '/follower_vehicle/odometry/filtered', self.odometry_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '/follower_vehicle/driver_inputs', 10)

        if self.predefined_path:
            self.sub_vehicle_state = self.create_subscription(Path, '/path', self.trajectory_callback, qos_profile)
        else:
            self.sub_vehicle_state = self.create_subscription(Path, '/leader_vehicle/vehicle_traj', self.trajectory_callback, qos_profile)
            self.sub_leader_odometry = self.create_subscription(Odometry, '/leader_vehicle/odometry/filtered', self.leader_odometry_callback, qos_profile)

        self.pub_path = self.create_publisher(Path, '/reference_path', 10)
        self.pub_odom_path = self.create_publisher(Path, '/follower_vehicle/vehicle_traj', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        
        # Service to start the node
        self.srv = self.create_service(Trigger, 'start_control', self.start_control_callback)

        # PID parameters for steering control
        self.steering_prev_cte = 0
        self.steering_cte_sum = 0
        self.steering_kp = -0.3
        self.steering_ki = 0.0
        self.steering_kd = 0.0
        self.steering_cte_integral_limit = 5.0
        
        # PID parameters for velocity control
        self.velocity_prev_error = 0
        self.velocity_error_sum = 0
        self.velocity_kp = 0.5
        self.velocity_ki = 0.1
        self.velocity_kd = 0.1
        self.velocity_error_integral_limit = 5.0
        self.target_velocity = 0.2  # Desired velocity (m/s)

    def trajectory_callback(self, msg):
        self.go = True
        self.start_publishing_path = True  # Start publishing path when this callback is triggered
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
        
        if self.start_publishing_path:
            self.update_odom_path()
    
    def leader_odometry_callback(self, msg):
        self.leader_x = msg.pose.pose.position.x
        self.leader_y = msg.pose.pose.position.y

    def update_odom_path(self):
        if len(self.odom_path) == 0 or self.distance(self.odom_path[-1], (self.x, self.y)) >= self.path_pub_distance_threshold:
            self.odom_path.append((self.x, self.y))
            if len(self.odom_path) > self.path_length_limit:
                self.odom_path.pop(0)
            self.publish_odom_path()

    def publish_odom_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (x, y) in self.odom_path:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0  # Assuming flat ground
            path_msg.poses.append(pose)

        self.pub_odom_path.publish(path_msg)
        
    def distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def error_state(self):
        x_current = self.x
        y_current = self.y
        theta_current = self.theta
        v_current = self.v

        dt = 1/self.freq
        
        # Post process theta
        while theta_current < -np.pi:
            theta_current += 2 * np.pi
        while theta_current > np.pi:
            theta_current -= 2 * np.pi

        dist = np.zeros((1, len(self.ref_traj)))
        for i in range(len(self.ref_traj)):
            dist[0][i] = (x_current + np.cos(theta_current) * self.lookahead - self.ref_traj[i][0]) ** 2 + (y_current + np.sin(theta_current) * self.lookahead - self.ref_traj[i][1]) ** 2
        index = dist.argmin()

        ref_state_current = self.ref_traj[index]
        
        dx = ref_state_current[0] - x_current
        dy = ref_state_current[1] - y_current
        ref_theta = np.arctan2(dy, dx)

        # Compute the angular error
        err_theta = ref_theta - theta_current

        # Normalize the angular error to be within [-pi, pi]
        while err_theta < -np.pi:
            err_theta += 2 * np.pi
        while err_theta > np.pi:
            err_theta -= 2 * np.pi
        
        # Compute cross-track error (CTE)
        self.cte = np.sin(err_theta) * np.sqrt(dx**2 + dy**2)
        
        # Compute cross-track error rate (CTE rate)
        self.cte_rate = (cte - self.steering_prev_cte) / dt

        self.steering_prev_cte = cte

        # CTE Integral term
        self.steering_cte_sum += cte * dt

        if self.steering_cte_sum > self.steering_cte_integral_limit:
            self.steering_cte_sum = self.steering_cte_integral_limit
        elif self.steering_cte_sum < -self.steering_cte_integral_limit:
            self.steering_cte_sum = -self.steering_cte_integral_limit

        error_state = [cte, cte_rate, self.steering_cte_sum]

        return error_state

    def compute_steering(self, error_state):
        steering = sum([x * y for x, y in zip(error_state, [self.steering_kp, self.steering_ki, self.steering_kd])])
        # Ensure steering can't change too much between timesteps, smooth transition
        delta_steering = steering - self.steering
        if abs(delta_steering) > 0.5:
            self.steering = self.steering + 0.5 * delta_steering / abs(delta_steering)
            self.get_logger().info("Steering changed too much, smoothing")
        else:
            self.steering = steering
        return np.clip(self.steering, -1.0, 1.0)

    def compute_throttle(self):
        velocity_error = self.target_velocity - self.v
        dt = 1 / self.freq
        velocity_error_rate = (velocity_error - self.velocity_prev_error) / dt
        self.velocity_prev_error = velocity_error

        self.velocity_error_sum += velocity_error * dt
        if self.velocity_error_sum > self.velocity_error_integral_limit:
            self.velocity_error_sum = self.velocity_error_integral_limit
        elif self.velocity_error_sum < -self.velocity_error_integral_limit:
            self.velocity_error_sum = -self.velocity_error_integral_limit

        throttle = self.velocity_kp * velocity_error + self.velocity_ki * self.velocity_error_sum + self.velocity_kd * velocity_error_rate
        return np.clip(throttle, 0, 1)

    # Callback to run a loop and publish data this class generates
    def pub_callback(self):
        if not self.go:
            return
        e = self.error_state()

        self.get_logger().info(f'Error State {e}.')

        self.steering = self.compute_steering(e)
        
        if not self.predefined_path:
            distance_to_leader = self.distance((self.x, self.y), (self.leader_x, self.leader_y))
            if distance_to_leader <= 1.0:
                self.throttle = 0.0
                self.get_logger().info("Within 1 meter of the leader vehicle, cutting throttle")
            else:
                self.throttle = self.compute_throttle()
        else:
            self.throttle = self.compute_throttle()
        
        ### For vehicle one
        msg_follower = VehicleInput()
        msg_follower.steering = self.steering
        msg_follower.throttle = self.throttle
        self.pub_vehicle_cmd.publish(msg_follower)
        
        self.get_logger().info(f"## Control Inputs {self.steering} {self.throttle}")
        self.get_logger().info(f"## Velocity {self.v} ")

    def publish_reference_path(self, origin, point):  # Publish a path straight to the reference point np.array([x,y,z]) from origin
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        # Implement
        
        self.pub_path.publish(path_msg)
    
    def start_control_callback(self, request, response):
        self.go = True
        response.success = True
        response.message = "Control node started"
        self.get_logger().info("Received start signal")
        return response

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control, executor)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
