import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from scipy.spatial import KDTree
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import time
from art_msgs.msg import VehicleInput


class NonLinearMPCNode(Node):
    def __init__(self):
        super().__init__('non_linear_mpc_node')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.kd_tree = None

        # Declare and cache parameters
        self.declare_parameter('leader_ns', "none")
        self.declare_parameter('robot_ns', "none")
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
        self.declare_parameter('throttle_range', 5.0)

        # Cache parameters
        self.leader_ns = self.get_parameter('leader_ns').get_parameter_value().string_value
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value
        self.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
        self.throttle_range = self.get_parameter('throttle_range').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.control_smoothing = self.get_parameter('control_smoothing').get_parameter_value().double_value
        self.collision_distance = self.get_parameter('collision_distance').get_parameter_value().double_value
        self.timer_frequency = self.get_parameter('timer_frequency').get_parameter_value().double_value

        path_topic = f'/{self.leader_ns}/vehicle_traj' if self.leader_ns != 'none' else '/path'
        leader_odom_topic = f'/{self.leader_ns}/odometry/filtered' if self.leader_ns != 'none' else None

        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile)
        self.path_sub = self.create_subscription(Path, "/path", self.go_callback, 10)
        self.traj_sub = self.create_subscription(Path, path_topic, self.path_callback, 10)
        self.leader_odom_sub = self.create_subscription(Odometry, leader_odom_topic, self.leader_odom_callback, qos_profile) if self.leader_ns != 'none' else None

        self.last_closest_index = 0
        self.cmd_pub = self.create_publisher(VehicleInput, '/input/driver_inputs', 10)
        self.trajectory_pub = self.create_publisher(Path, '/local_mpc', 30)
        self.reference_path_pub = self.create_publisher(Path, f'{self.robot_ns}/reference_path', 10)

        self.current_pose = None
        self.leader_pose = None
        self.reference_path = []
        self.current_speed = 0.0

        self.previous_steering = 0.0
        self.previous_control_sequence = np.zeros((self.horizon, 2))
        self.previous_control = np.zeros(2)
        self.path_received = False

        # ACADOS Setup
        self.solver = self.setup_acados_solver()

        self.timer = self.create_timer(self.timer_frequency, self.timer_callback)

        self.go = False

    def normalize_angle(self, angle):
        """Normalize the angle to be within the range [-pi, pi]."""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def setup_acados_solver(self):
        ocp = AcadosOcp()
        model = self.define_acados_model()
        ocp.model = model

        # Set prediction horizon
        ocp.dims.N = self.horizon
        nx = 10  # Numb. of state variables in Obj func
        nu = 2 # Numb. of control variables in Obj func
        ny = nx + nu
        ny_e = nx

        # Define cost matrices
        # states = ca.vertcat(x, y, theta, v, delta_x, delta_y, delta_heading, delta_steering, distance_to_leader,v_err)

        Q = np.diag([0, 0, 0, 0, 1800, 1800, 800, 300, 25, 1000])  # Adding penalty for distance_to_leader
        R = np.diag([400, 3000])  # a, delta
        Qe = Q

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        W = np.zeros((ny, ny))
        W[:nx, :nx] = Q
        W[nx:, nx:] = R
        ocp.cost.W = W
        ocp.cost.W_e = Qe

        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        Vu = np.zeros((ny, nu))
        Vu[nx:, :] = np.eye(nu)
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        ocp.constraints.lbu = np.array([0, -self.max_steering_angle])
        ocp.constraints.ubu = np.array([self.max_acceleration, self.max_steering_angle])
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.x0 = np.zeros(nx)

        if(self.leader_ns != 'none'):
            ocp.parameter_values = np.zeros(5)  # Including leader's position if necessary
        else:
            ocp.parameter_values = np.zeros(3)

        ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'IRK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.tf = self.dt * self.horizon
        ocp.solver_options.generate_hessian = True
        ocp.solver_options.qp_solver_iter_max = 300
        ocp.solver_options.regularization_method = 'CONVEXIFY'
        ocp.solver_options.nlp_solver_warm_start = True

        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 4

        return AcadosOcpSolver(ocp, json_file=f"acados_ocp_{self.leader_ns=='none'}.json")

    def define_acados_model(self):
        model = AcadosModel()
        model.name = f"mpc_model_{self.leader_ns=='none'}"

        # Define the states
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        v = ca.SX.sym('v')
        delta_x = ca.SX.sym('delta_x')
        delta_y = ca.SX.sym('delta_y')
        delta_heading = ca.SX.sym('delta_heading')
        distance_to_leader = ca.SX.sym('distance_to_leader')
        delta_steering = ca.SX.sym('delta_steering')
        v_err = ca.SX.sym('v_err')
        states = ca.vertcat(x, y, theta, v, delta_x, delta_y, delta_heading, delta_steering, distance_to_leader,v_err)

        # Define the state derivatives
        x_dot = ca.SX.sym('x_dot')
        y_dot = ca.SX.sym('y_dot')
        theta_dot = ca.SX.sym('theta_dot')
        v_dot = ca.SX.sym('v_dot')
        delta_x_dot = ca.SX.sym('delta_x_dot')
        delta_y_dot = ca.SX.sym('delta_y_dot')
        delta_heading_dot = ca.SX.sym('delta_heading_dot')
        distance_to_leader_dot = ca.SX.sym('distance_to_leader_dot')
        delta_steering_dot = ca.SX.sym('delta_steering_dot')
        v_err_dot = ca.SX.sym('v_err_dot')
        xdot = ca.vertcat(x_dot, y_dot, theta_dot, v_dot, delta_x_dot, delta_y_dot, delta_heading_dot, delta_steering_dot, distance_to_leader_dot,v_err_dot)

        # Define the control inputs
        a = ca.SX.sym('a')
        delta = ca.SX.sym('delta')
        controls = ca.vertcat(a, delta)

        # Define the reference parameters (x_ref, y_ref, theta_ref)
        x_ref = ca.SX.sym('x_ref')
        y_ref = ca.SX.sym('y_ref')
        theta_ref = ca.SX.sym('theta_ref')

        # Leader reference positions (optional)
        if self.leader_ns != 'none':
            leader_x = ca.SX.sym('leader_x')
            leader_y = ca.SX.sym('leader_y')
            parameters = ca.vertcat(x_ref, y_ref, theta_ref, leader_x, leader_y)

            # Calculate distance to leader squared
            distance_to_leader_diff = ((leader_x - x)**2 + (leader_y - y)**2)
        else:
            parameters = ca.vertcat(x_ref, y_ref, theta_ref)
            distance_to_leader_diff = distance_to_leader

        # Normalize angle using CasADi's symbolic operations
        theta_diff = theta - theta_ref
        normalized_theta_diff = ca.fabs(ca.atan2(ca.sin(theta_diff), ca.cos(theta_diff)))

        # Define the explicit dynamics
        f_expl = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            (v / self.wheelbase) * ca.tan(delta),
            a - (0.01 * ca.fabs(v * v)) - (0.5 * ca.fabs(v)),
            # # # # # # # # # #
            delta_steering,
            ca.fabs(x_ref - x),
            ca.fabs(y_ref - y),
            normalized_theta_diff,
            distance_to_leader_diff - self.collision_distance, # Add trailing dist
            ca.fmax(v - self.max_speed,0)
        )

        # Define the implicit dynamics F(xdot, x, u) = 0
        f_impl = xdot - f_expl

        # Set model equations
        model.f_impl_expr = f_impl
        model.x = states
        model.xdot = xdot
        model.u = controls
        model.p = parameters

        return model

    def solve_mpc(self, current_state, reference_path):
        if not reference_path:
            return None, np.zeros(2)

        x, y, theta, v = current_state[:4]
        _, closest_index = self.kd_tree.query(current_state[:2])

        remaining_path_length = len(reference_path) - closest_index
        adjusted_horizon = min(self.horizon, remaining_path_length)

        path_segment = []
        for pose in reference_path[closest_index:closest_index + adjusted_horizon]:
            x_ref, y_ref, theta_ref = pose
            theta_ref = self.normalize_angle(theta_ref)
            path_segment.append([x_ref, y_ref, theta_ref])

        self.publish_reference_path(path_segment)

        for i in range(adjusted_horizon):
            if i < len(self.previous_control_sequence):
                self.solver.set(i, "u", self.previous_control_sequence[i])
            else:
                self.solver.set(i, "u", np.zeros(2))

        for i in range(adjusted_horizon):
            if i < len(path_segment):
                path_seg = path_segment[i]
            else:
                path_seg = reference_path[-1][:3]

            x_ref, y_ref, theta_ref = path_seg
            if i < len(self.previous_control_sequence) - 1:
                d_control = self.previous_control_sequence[i+1] - self.previous_control_sequence[i]

            if self.leader_pose is not None:
                leader_x, leader_y = self.leader_pose[:2]
                yaw = self.leader_pose[2]
                v = self.leader_speed 

                leader_vx = v * np.cos(yaw)
                leader_vy = v * np.sin(yaw)

                leader_x = leader_x + (leader_vx * self.dt * i)
                leader_y = leader_y + (leader_vy * self.dt * i)

                parameters = np.array([x_ref, y_ref, theta_ref, leader_x, leader_y])
            else:
                parameters = np.array([x_ref, y_ref, theta_ref])

            ref_state = np.array([x_ref, y_ref, theta_ref, self.max_speed, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

            self.solver.set(i, "p", parameters)

            if i < len(self.previous_control_sequence):
                ref_control = self.previous_control_sequence[i]
            else:
                ref_control = np.zeros(2)

            yref_i = np.concatenate([ref_state, ref_control])
            self.solver.set(i, "yref", yref_i)

        if(i < len(self.previous_control_sequence)-1):
            d_steering = self.previous_control_sequence[i][1] - self.previous_control_sequence[i-1][1]
        else:
            d_steering = 0.0
        current_state_with_diff = np.append(current_state, [d_steering, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.solver.set(0, "lbx", current_state_with_diff)
        self.solver.set(0, "ubx", current_state_with_diff)

        status = self.solver.solve()

        if status != 0:
            self.previous_control_sequence = [np.zeros(2) for _ in range(adjusted_horizon)]
            return None, np.zeros(2)

        u_opt = self.solver.get(0, "u")
        self.previous_control_sequence = [self.solver.get(i, "u") for i in range(adjusted_horizon)]
        trajectory = [self.solver.get(i, "x") for i in range(adjusted_horizon + 1)]

        return trajectory, u_opt

    def go_callback(self, msg):
        self.go = True

    def odom_callback(self, msg):
        self.current_pose = np.array([msg.pose.pose.position.x,
                                      msg.pose.pose.position.y,
                                      self.get_yaw_from_quaternion(msg.pose.pose.orientation)])
        self.current_speed = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def leader_odom_callback(self, msg):
        self.leader_pose = np.array([msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     self.get_yaw_from_quaternion(msg.pose.pose.orientation)])
        self.leader_speed = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

    def path_callback(self, msg):
        self.reference_path = [np.array([pose.pose.position.x, pose.pose.position.y, self.get_yaw_from_quaternion(pose.pose.orientation)]) for pose in msg.poses]
        self.kd_tree = KDTree([point[:2] for point in self.reference_path])
        self.path_received = True

    def timer_callback(self):
        if not self.go:
            return

        if self.current_pose is not None and self.reference_path:
            current_state = np.append(self.current_pose, self.current_speed)

            optimal_trajectory, control = self.solve_mpc(current_state, self.reference_path)

            self.publish_control(control)
            self.publish_trajectory(optimal_trajectory)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return self.normalize_angle(yaw)

    def publish_reference_path(self, reference_path, leader_distance=None):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for state in reference_path:
            pose = PoseStamped()
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation = self.get_quaternion_from_yaw(state[2])
            path_msg.poses.append(pose)

        self.reference_path_pub.publish(path_msg)

        if leader_distance is not None:
            self.get_logger().info(f"Distance to leader: {leader_distance:.2f}")

    def convert_acceleration_to_throttle(self, acceleration, max_acceleration, min_throttle=0.0, max_throttle=1.0):
        if acceleration > max_acceleration:
            acceleration = max_acceleration
        elif acceleration < 0:
            acceleration = 0
        throttle = (acceleration / max_acceleration) * (max_throttle - min_throttle) + min_throttle
        return throttle

    def publish_control(self, control):
        if not self.go:
            return

        throttle = self.convert_acceleration_to_throttle(control[0], self.throttle_range)

        steering_angle = control[1]
        steering_input = steering_angle / self.max_steering_angle

        if abs(steering_input - self.previous_steering) > self.control_smoothing:
            steering_input = self.previous_steering + np.sign(steering_input - self.previous_steering) * self.control_smoothing

        self.previous_steering = steering_input

        msg_follower = VehicleInput()
        msg_follower.steering = steering_input
        msg_follower.throttle = throttle

        self.cmd_pub.publish(msg_follower)

    def publish_trajectory(self, trajectory):
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        if trajectory is None:
            return

        for state in trajectory:
            pose = PoseStamped()
            pose.pose.position.x = state[0]
            pose.pose.position.y = state[1]
            pose.pose.orientation = self.get_quaternion_from_yaw(self.normalize_angle(state[2]))
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
