import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, Path
from art_msgs.msg import VehicleInput  # Replace with the actual import path for VehicleInput
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy
import time
from scipy.spatial import KDTree
import casadi as ca

class NonLinearMPCNode(Node):
    def __init__(self):
        super().__init__('non_linear_mpc_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.kd_tree = None  # Initialize the kd-tree to None

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
        self.declare_parameter('timer_frequency', 0.05)

        self.leader_ns = self.get_parameter('leader_ns').get_parameter_value().string_value
        path_topic = f'/{self.leader_ns}/vehicle_traj' if self.leader_ns != 'none' else '/path'
        leader_odom_topic = f'/{self.leader_ns}/odometry/filtered' if self.leader_ns != 'none' else None

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile)
        self.path_sub = self.create_subscription(Path, "/path", self.go_callback, 10)
        self.traj_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.leader_odom_sub = self.create_subscription(Odometry, leader_odom_topic, self.leader_odom_callback, qos_profile) if self.leader_ns != 'none' else None
        self.last_closest_index = 0  # Initialize in the constructor

        self.cmd_pub = self.create_publisher(VehicleInput, '/input/driver_inputs', 10)  # Update topic if necessary
        self.trajectory_pub = self.create_publisher(Path, '/local_mpc', 30)  # Publisher for the local MPC path
        self.reference_path_pub = self.create_publisher(Path, '/reference_path', 10)

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

        self.timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_frequency, self.timer_callback)

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
        self.kd_tree = KDTree([point[:2] for point in self.reference_path])  # Create the kd-tree using the 2D positions of the reference path
        self.path_received = True

    def timer_callback(self):
        if not self.go:
            self.get_logger().info("Waiting for /path to be published...")
            return

        if self.current_pose is not None and self.reference_path:
            current_state = np.append(self.current_pose, self.current_speed)  # Add speed to the current state
            
            start_time = time.time()  # Start timer
            optimal_trajectory, control = self.solve_mpc(current_state, self.reference_path)
            end_time = time.time()  # End timer
            
            # Check if optimization took longer than the timer frequency
            duration = end_time - start_time
            if duration > self.timer_frequency:
                self.get_logger().warn(f"Optimization took longer ({duration:.3f} seconds) than the timer frequency ({self.timer_frequency} seconds)")

            self.publish_control(control)
            self.publish_trajectory(optimal_trajectory)
    
    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)
        


    def solve_mpc(self, current_state, reference_path):
        if not reference_path:
            self.get_logger().warn("Reference path is empty!")
            return None, np.zeros(2)
            
        horizon = self.horizon
        dt = self.dt

        # Find the closest point on the path to the current position
        closest_index = self.find_closest_point(current_state, self.kd_tree)

        # Shift the reference path to start from the closest point
        reference_path_horizon = reference_path[closest_index:closest_index+horizon]

        if len(reference_path_horizon) < horizon:
            reference_path_horizon = reference_path_horizon + [reference_path[-1]] * (horizon - len(reference_path_horizon))

        # Publish the reference path for debug purposes
        self.publish_reference_path(reference_path_horizon)

        opti = ca.Opti()

        u = opti.variable(horizon, 2)  # Control inputs: throttle and steering angle
        x = opti.variable(horizon+1, 4) # State variables: x, y, theta, v

        current_state = current_state.reshape((1, -1))

        opti.subject_to(x[0, :] == current_state)

        J = 0
        for i in range(horizon):
            state = x[i, :].T
            control = u[i, :].T
            next_state = self.vehicle_dynamics(state, control).T

            opti.subject_to(x[i+1, :] == next_state)

            # Reference state
            if i < len(reference_path_horizon) - 1:
                ref_state = reference_path_horizon[i]
                next_ref_state = reference_path_horizon[i + 1]
            else:
                ref_state = reference_path_horizon[-1]
                next_ref_state = reference_path_horizon[-1]

            # Compute cross-track and heading errors
            cross_track_error = self.compute_cross_track_error(state, ref_state, next_ref_state)
            heading_error = ca.arctan2(ca.sin(ref_state[2] - state[2]), ca.cos(ref_state[2] - state[2]))

            # Cost function penalizes cross-track and heading errors
            J += 1000 * ca.power(cross_track_error, 2)
            J += 200 * ca.power(heading_error, 2)

            J += ca.if_else(state[3] < self.min_speed, 250 * ca.power(self.min_speed - state[3], 2), 0.0)
            J += ca.if_else(state[3] > self.max_speed, 250 * ca.power(state[3] - self.max_speed, 2), 0.0)

            # Penalize control input changes
            if i > 0:
                p = ca.power(control - u[i-1, :].T, 2)
                J += 50 * (p[0] + p[1])
             
        opti.minimize(J)
        opti.subject_to(opti.bounded(0.0, u[:, 0], 1.0))  # Throttle constraints
        opti.subject_to(opti.bounded(-0.5, u[:, 1], 0.5))  # Steering constraints

        opts = {'ipopt.print_level': 0, 'print_time': 0}
        opti.solver('ipopt', opts)

        # Set the initial guess for the control sequence to the last optimal sequence
        opti.set_initial(u, self.previous_control_sequence)
        opti.set_initial(x, self.generate_trajectory(current_state.flatten(), self.previous_control_sequence))

        try:
            sol = opti.solve()
            optimal_control_sequence = sol.value(u)
            self.previous_control_sequence = optimal_control_sequence
            optimal_trajectory = self.generate_trajectory(current_state.flatten(), optimal_control_sequence)
        except RuntimeError as e:
            self.get_logger().warn(f"MPC solver failed to find a solution: {e}")

            x_values = opti.debug.value(x)
            u_values = opti.debug.value(u)
            self.get_logger().info(f"x_values: {x_values}")
            self.get_logger().info(f"u_values: {u_values}")

            optimal_control_sequence = self.previous_control_sequence
            optimal_trajectory = self.generate_trajectory(current_state.flatten(), optimal_control_sequence)

        return optimal_trajectory, optimal_control_sequence[0]

    def publish_reference_path(self, reference_path):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Adjust frame_id as necessary
        
        for state in reference_path:
            pose = PoseStamped()
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation = self.get_quaternion_from_yaw(state[2])
            path_msg.poses.append(pose)
        
        self.reference_path_pub.publish(path_msg)

    # def compute_cross_track_error(self, state, ref_state, next_ref_state):
    #     ref_point = ref_state[:2]
    #     next_ref_point = next_ref_state[:2]
    #     state_point = state[:2]

    #     v1 = ca.reshape(next_ref_point - ref_point, (1, -1)).T  

    #     v2 = state_point.T - ref_point

        
    #     # Manually compute the 2D cross product
    #     cross_track_error = ca.fabs(v1[0] * v2[1] - v1[1] * v2[0]) / ca.norm_2(v1)

    #     return cross_track_error

    def compute_cross_track_error(self, state, ref_state, next_ref_state):
        ref_point = ref_state[:2]
        next_ref_point = next_ref_state[:2]
        state_point = state[:2]

        v1 = next_ref_point - ref_point  # Reference direction vector
        v2 = state_point - ref_point  # State to reference start vector

        # Cross product in 2D to find the area of the parallelogram
        cross_product = v1[0] * v2[1] - (v1[1] * v2[0])
        eps = 1e-10
        error = cross_product / (np.linalg.norm(v1)+eps)

        return error

    def find_closest_point(self, current_pose, tree):
        if tree is not None:
            # Ensure current_pose is numeric
            current_pose_numeric = np.array([current_pose[0], current_pose[1]])
            _, index = tree.query(current_pose_numeric)
            return index
        else:
            return 0

    def generate_trajectory(self, initial_state, control_sequence):
        trajectory = [ca.DM(initial_state)]  # Start with a DM type
        state = initial_state

        for control in control_sequence:
            state = self.vehicle_dynamics(state, control).full().flatten()
            trajectory.append(ca.DM(state))  # Convert each state to a DM type

        return ca.horzcat(*trajectory).T  # Return as a DM matrix with the correct shape


    def vehicle_dynamics(self, state, control):
        x, y, theta, v = state[0], state[1], state[2], state[3]
        throttle, delta = control[0], control[1]
        L = self.get_parameter('wheelbase').get_parameter_value().double_value  # Wheelbase
        max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value  # Maximum acceleration in m/s^2
        
        # Update speed with throttle input
        a = throttle * max_acceleration

        v_next = v + a * self.dt - (0.1)
        
        x_next = x + v * ca.cos(theta) * self.dt
        y_next = y + v * ca.sin(theta) * self.dt
        theta_next = theta + ((v / L) * ca.tan(delta) * self.dt)
        
        return ca.vertcat(x_next, y_next, theta_next, v_next)

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
                msg_follower.braking = 1 / distance_to_leader
            else:
                msg_follower.braking = 0.0
        
        self.cmd_pub.publish(msg_follower)
        

    def publish_trajectory(self, trajectory):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # Adjust frame_id as necessary
        
        # Convert CasADi matrix to a list of NumPy arrays for iteration
        trajectory_list = np.array(trajectory.full())

        for state in trajectory_list:
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
    

def main(args=None):
    rclpy.init(args=args)
    node = NonLinearMPCNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
