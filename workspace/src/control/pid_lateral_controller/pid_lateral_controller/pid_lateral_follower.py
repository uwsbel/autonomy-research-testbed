import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
import numpy as np

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

class PIDLateralFollowerNode(Node):
    """A PID controller.

    This node subscribes to the state of the target and current vehicles and publishes vehicle inputs to follow the target vehicle.

    Attributes:
        mode: The control mode to be used (currently only PID)
        steering_gain: The gain for the steering input
        throttle_gain: The gain for the throttle input
    """

    def __init__(self):
        super().__init__("pid_lateral_controller_node")

        # DEFAULT SETTINGS

        # pid_lateral_controller node mode
        self.mode = "PID"

        # update frequency of this node
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # ROS PARAMETERS
        self.declare_parameter("control_mode", "PID")
        self.mode = (
            self.get_parameter("control_mode").get_parameter_value().string_value
        )

        self.declare_parameter("steering_gain", 1.0)
        self.steering_gain = (
            self.get_parameter("steering_gain").get_parameter_value().double_value
        )
        self.declare_parameter("throttle_gain", 1.0)
        self.throttle_gain = (
            self.get_parameter("throttle_gain").get_parameter_value().double_value
        )

        self.declare_parameter("lookahead_distance", 2.0)
        self.lookahead_distance = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )

        self.declare_parameter("leader_ns")
        self.leader_ns = (
            self.get_parameter("leader_ns").get_parameter_value().string_value
        )

        self.declare_parameter("robot_ns")
        self.robot_ns = (
            self.get_parameter("robot_ns").get_parameter_value().string_value
        )

        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0

        # data that will be used by this class
        self.state = ""
        self.target_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.vehicle_cmd = VehicleInput()

        # waits for first state data
        self.go = False

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_target_pose = self.create_subscription(
            PoseStamped, f"/{self.leader_ns}/output/vehicle/state/pose", self.target_pose_callback, qos_profile
        )
        self.sub_current_pose = self.create_subscription(
            PoseStamped, f"/{self.robot_ns}/output/vehicle/state/pose", self.current_pose_callback, qos_profile
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, f"/{self.robot_ns}/input/driver_inputs", 10
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    def target_pose_callback(self, msg):
        """Callback for the target vehicle pose subscriber.

        Read the pose of the target vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.go = True
        self.target_pose = msg
        # self.get_logger().info(f"Received target pose: {msg}")

    def current_pose_callback(self, msg):
        """Callback for the current vehicle pose subscriber.

        Read the pose of the current vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.current_pose = msg
        # self.get_logger().info(f"Received current pose: {msg}")

    def pub_callback(self):
        """Callback for the publisher.

        Publish the vehicle inputs to follow the target vehicle.
        """
        if not self.go:
            return

        msg = VehicleInput()

        try:
            # Compute the heading error with lookahead
            target_position = self.target_pose.pose.position
            current_position = self.current_pose.pose.position
            target_orientation = self.target_pose.pose.orientation
            rotation = self.current_pose.pose.orientation


            # Offset target position to be 1m behind the target vehicle
            q = [target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w]
            target_yaw = np.arctan2(2 * (q[1] * q[2] + q[3] * q[0]), q[3]**2 + q[0]**2 - q[1]**2 - q[2]**2)
            target_position.x -= np.cos(target_yaw)
            target_position.y -= np.sin(target_yaw)

            # Compute the heading to the target 
            dx = target_position.x - current_position.x
            dy = target_position.y - current_position.y
            heading_to_target = np.arctan2(dy, dx)

            # # Extract rotation (quaternion) from current pose
            x = rotation.x
            y = rotation.y
            z = rotation.z
            w = rotation.w
            yaw = np.arctan2(2 * (y * z + w * x), w**2 + x**2 - y**2 - z**2)

            heading_error = heading_to_target 
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error)) - yaw
            steering = self.steering_gain * heading_error

            # Ensure steering can't change too much between timesteps, smooth transition
            delta_steering = steering - self.steering
            if abs(delta_steering) > 0.1:
                self.steering += 0.1 * delta_steering / abs(delta_steering)
            else:
                self.steering = steering

            # Proportional throttle
            distance = np.sqrt(dx ** 2 + dy ** 2)    
            self.throttle = self.throttle_gain * distance / 5.0

            # Hacky, but makes for much better turning with large steering angles
            if(abs(self.steering) >= 0.8):
                self.throttle = min(self.throttle, 0.2)


            msg.steering = np.clip(self.steering, -1, 1)
            msg.throttle = np.clip(self.throttle, 0, 1)
            msg.braking = np.clip(self.braking, 0, 1)
            msg.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(f"Publishing vehicle command: {msg}")
            self.get_logger().info(f"Current state: steering={msg.steering}, throttle={msg.throttle}, braking={msg.braking}")
            self.get_logger().info(f"....: dx={dx}, dy={dy}, heading_error={heading_error} yaw={yaw} target_yaw={target_yaw}")
            self.pub_vehicle_cmd.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to compute vehicle command: {e}")

def main(args=None):
    rclpy.init(args=args)
    control = PIDLateralFollowerNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
