import rclpy
from rclpy.node import Node
from scipy.optimize import minimize
import numpy as np
from nav_msgs.msg import Odometry, Path
from art_msgs.msg import VehicleInput  # Replace with the actual import path for VehicleInput
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class NonLinearMPCNode(Node):
    def __init__(self):
        super().__init__('non_linear_mpc_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.declare_parameter('leader_ns', "none")
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.2)
        self.declare_parameter('horizon', 5)
        self.declare_parameter('dt', 0.4)
        self.declare_parameter('max_acceleration', 5.0)
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('control_smoothing', 0.25)
        self.declare_parameter('collision_distance', 3.0)

        self.leader_ns = self.get_parameter('leader_ns').get_parameter_value().string_value
        path_topic = f'/{self.leader_ns}/vehicle_traj' if self.leader_ns != 'none' else '/path'
        leader_odom_topic = f'/{self.leader_ns}/odometry/filtered' if self.leader_ns != 'none' else None

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile)
        self.path_sub = self.create_subscription(Path, "/path", self.go_callback, 10)
        self.traj_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.leader_odom_sub = self.create_subscription(Odometry, leader_odom_topic, self.leader_odom_callback, qos_profile) if self.leader_ns != 'none' else None

        self.cmd_pub = self.create_publisher(VehicleInput, '/input/driver_inputs', 10)  # Update topic if necessary
        self.trajectory_pub = self.create_publisher(Path, '/local_mpc', 30)  # Publisher for the local MPC path
        
        self.current_pose = None
        self.leader_pose = None
        self.reference_path = []
        self.current_speed = 0.0
        
        self.previous_steering = 0.0  # Initialize previous steering angle

        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value

        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        self.previous_control_sequence = np.zeros((self.horizon, 2))  # Initialize previous control sequence
        
        self.path_received = False
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.go = False
        
    def go_callback(self, msg):
        self.go = True

    def odom_callback(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    self.get_yaw_from_quaternion(msg.pose.pose.orientation)])
        self.current_speed = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.get_logger().info(f"Current speed: {self.current_speed}")
    
    def leader_odom_callback(self, msg):
        self.leader_pose = np.array([msg.pose.pose.position.x,
                                    msg.pose.pose.position.y,
                                    self.get_yaw_from_quaternion(msg.pose.pose.orientation)])
        self.leader_speed = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self.get_logger().info(f"Leader speed: {self.leader_speed}")
            
    def path_callback(self, msg):
        self.reference_path = [np.array([pose.pose.position.x, pose.pose.position.y, self.get_yaw_from_quaternion(pose.pose.orientation)]) for pose in msg.poses]
        self.path_received = True
    
    def timer_callback(self):
        if not self.go:
            self.get_logger().info("Waiting for /path to be published...")
            return

        if self.current_pose is not None and self.reference_path:
            closest_index = self.find_closest_point(self.current_pose, self.reference_path)
            local_reference_path = self.reference_path[closest_index:]
            current_state = np.append(self.current_pose, self.current_speed)  # Add speed to the current state
            optimal_trajectory, control = self.solve_mpc(current_state, local_reference_path)
            self.publish_control(control)
            self.publish_trajectory(optimal_trajectory)
    
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def vehicle_dynamics(self, state, control):
        x, y, theta, v = state
        throttle, delta = control
        L = self.get_parameter('wheelbase').get_parameter_value().double_value  # Wheelbase
        max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value  # Maximum acceleration in m/s^2
        
        # Update speed with throttle input
        a = throttle * max_acceleration
        v_next = v + a * self.dt - (0.1)
        
        x_next = x + v * np.cos(theta) * self.dt
        y_next = y + v * np.sin(theta) * self.dt
        theta_next = theta + (v / L) * np.tan(delta) * self.dt
        
        return np.array([x_next, y_next, theta_next, v_next])

    def cost_function(self, control_sequence, *args):
        state, reference_path = args
        cost = 0.0
        
        control_sequence = control_sequence.reshape(-1, 2)
        
        for i, control in enumerate(control_sequence):
            state = self.vehicle_dynamics(state, control)
            closest_ref_index = self.find_closest_point(state, reference_path)
            
            if closest_ref_index < len(reference_path):
                # Compare only position and orientation
                ref_state = reference_path[closest_ref_index]
                cross_track_error = np.linalg.norm(np.cross(ref_state[:2] - state[:2], ref_state[:2] - reference_path[min(closest_ref_index+1, len(reference_path)-1)][:2]) / np.linalg.norm(ref_state[:2] - reference_path[min(closest_ref_index+1, len(reference_path)-1)][:2]))
                heading_error = np.arctan2(np.sin(ref_state[2] - state[2]), np.cos(ref_state[2] - state[2]))  # Calculate heading error
                cost += 100 * cross_track_error**2  # Adjust the weight as necessary
                cost += 30 * heading_error**2  # Adjust the weight as necessary
            else:
                cost += np.sum((state[:3] - reference_path[-1][:3])**2)
            
            # Penalize velocities outside the desired range
            if state[3] < self.min_speed:
                cost += 1000 * (self.min_speed - state[3])**2
            elif state[3] > self.max_speed:
                cost += 1000 * (state[3] - self.max_speed)**2

            # Ensure follower's velocity does not exceed leader's velocity if within 2 meters
            if self.leader_pose is not None:
                distance_to_leader = np.linalg.norm(state[:2] - self.leader_pose[:2])
                # if distance_to_leader > 6.0:
                    # cost += 1000 * (distance_to_leader - 6.0)**2
                if distance_to_leader < 2.0:
                    cost += 10000 * (state[3] - self.leader_speed)**2  # Heavy penalty for exceeding leader's speed
                # if distance_to_leader < 1.0:
                # cost += 2500 * distance_to_leader**2  # Heavy penalty for being within 2 meters

            # Penalize rapid changes in control inputs for smoothness
            if i > 0:
                cost += 500 * np.sum((control - control_sequence[i-1])**2)
            
            # Penalize if follower is too close to the leader
            # if self.leader_pose is not None:
            #     distance_to_leader = np.linalg.norm(state[:2] - self.leader_pose[:2])
            #     if distance_to_leader < 1.0:  # 3 meter safety distance
            #         cost += 10000 * (1.0 - distance_to_leader)**2

        return cost    
        
    def solve_mpc(self, current_state, reference_path):
        control_sequence_init = self.previous_control_sequence.flatten()  # Use the previous control sequence as the initial guess
        
        bounds = [(0.0, 1.0), (-0.5, 0.5)] * self.horizon
        
        result = minimize(self.cost_function, control_sequence_init, args=(current_state, reference_path),
                          bounds=bounds, method='SLSQP')
        
        optimal_control_sequence = result.x.reshape(-1, 2)
        self.previous_control_sequence = optimal_control_sequence  # Store the optimal control sequence for the next iteration
        optimal_trajectory = self.generate_trajectory(current_state, optimal_control_sequence)
        
        return optimal_trajectory, optimal_control_sequence[0]
    
    def generate_trajectory(self, initial_state, control_sequence):
        trajectory = [initial_state]
        state = initial_state
        
        for control in control_sequence:
            state = self.vehicle_dynamics(state, control)
            trajectory.append(state)
        
        return trajectory

    def publish_control(self, control):
        steering_angle = control[1]
        max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value  # Corresponds to a steering input of 1
        steering_input = steering_angle / max_steering_angle
        
        # Smooth steering input
        control_smoothing = self.get_parameter('control_smoothing').get_parameter_value().double_value
        if abs(steering_input - self.previous_steering) > control_smoothing:
            steering_input = self.previous_steering + np.sign(steering_input - self.previous_steering) * control_smoothing
        
        self.previous_steering = steering_input
        
        msg_follower = VehicleInput()
        msg_follower.steering = steering_input  # Scale the steering angle to steering input
        msg_follower.throttle = control[0]  # Assuming control[0] is throttle
        
        # Add braking if the distance to leader is less than the specified distance
        collision_distance = self.get_parameter('collision_distance').get_parameter_value().double_value
        if self.leader_pose is not None:
            distance_to_leader = np.linalg.norm(self.current_pose[:2] - self.leader_pose[:2])
            if distance_to_leader < collision_distance:
                msg_follower.braking = 1/distance_to_leader
            else:
                msg_follower.braking = 0.0
        
        self.cmd_pub.publish(msg_follower)
        
    
    def publish_trajectory(self, trajectory):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Adjust frame_id as necessary
        
        for state in trajectory:
            pose = PoseStamped()
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation = self.get_quaternion_from_yaw(state[2])
            path_msg.poses.append(pose)
        
        self.trajectory_pub.publish(path_msg)
    
    def get_quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.z = np.sin(yaw / 2)
        q.w = np.cos(yaw / 2)
        return q
    
    def find_closest_point(self, current_pose, path):
        dists = [np.linalg.norm(current_pose[:2] - point[:2]) for point in path]
        return np.argmin(dists)

def main(args=None):
    rclpy.init(args=args)
    node = NonLinearMPCNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
