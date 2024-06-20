import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from geometry_msgs.msg import Point
import numpy as np

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

class PurePursuitFollowerNode(Node):
    """A Pure Pursuit controller.

    This node subscribes to the state of the target and current vehicles and publishes vehicle inputs to follow the target vehicle.

    Attributes:
        lookahead_distance: The distance ahead to look for the target point
        steering_gain: The gain for the steering input
        throttle_gain: The gain for the throttle input
    """

    def __init__(self):
        super().__init__("pure_pursuit_controller_node")

        # DEFAULT SETTINGS

        # update frequency of this node
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # ROS PARAMETERS
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

        self.declare_parameter("leader_ns", "")
        self.leader_ns = (
            self.get_parameter("leader_ns").get_parameter_value().string_value
        )

        self.declare_parameter("robot_ns", "")
        self.robot_ns = (
            self.get_parameter("robot_ns").get_parameter_value().string_value
        )

        if self.leader_ns == "none":
            self.get_logger().info("Leader namespace set to 'none', shutting down node.")
            rclpy.shutdown()
            return

        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0
        self.target_velocity = 0.0

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
        self.sub_target_twist = self.create_subscription(
            TwistStamped, f"/{self.leader_ns}/output/vehicle/state/twist", self.target_twist_callback, qos_profile
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, f"/{self.robot_ns}/input/driver_inputs", 10
        )

        self.timer = self.create_timer(1. / 10., self.pub_callback)

    def target_pose_callback(self, msg):
        """Callback for the target vehicle pose subscriber.

        Read the pose of the target vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.go = True
        self.target_pose = msg

    def current_pose_callback(self, msg):
        """Callback for the current vehicle pose subscriber.

        Read the pose of the current vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.current_pose = msg
        self.pub_callback()

    def target_twist_callback(self, msg):
        """Callback for the target vehicle twist subscriber.

        Read the twist (velocity) of the target vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.target_velocity = msg.twist.linear.x

    def pub_callback(self):
        """Callback for the publisher.

        Publish the vehicle inputs to follow the target vehicle.
        """

        if not self.go:
            return

        msg = VehicleInput()

        try:
            # Compute the Pure Pursuit control

            target_position = self.target_pose.pose.position
            current_position = self.current_pose.pose.position
            target_orientation = self.target_pose.pose.orientation
            current_orientation = self.current_pose.pose.orientation

            x = current_orientation.x
            y = current_orientation.y
            z = current_orientation.z
            w = current_orientation.w
            yaw = np.arctan2(2 * (y * z + w * x), w ** 2 + x ** 2 - y ** 2 - z ** 2)


            # Compute the lookahead point
            dx = target_position.x - current_position.x
            dy = target_position.y - current_position.y
            distance_to_target = np.sqrt(dx ** 2 + dy ** 2)

            lookahead_point = Point()
            if distance_to_target < self.lookahead_distance:
                lookahead_point = target_position
            else:
                lookahead_ratio = self.lookahead_distance / distance_to_target
                lookahead_point.x = current_position.x + lookahead_ratio  * dx #* np.cos(yaw)
                lookahead_point.y = current_position.y + lookahead_ratio * dy #* np.sin(yaw) 

            # Compute the heading to the lookahead point
            dx = lookahead_point.x - current_position.x
            dy = lookahead_point.y - current_position.y
            heading_to_lookahead = np.arctan2(dy, dx)

  
            heading_error = heading_to_lookahead - yaw
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
            self.steering = self.steering_gain * heading_error

            # Proportional throttle based on distance to target and target velocity
            self.throttle = 0.8 * self.throttle_gain * distance_to_target #(distance_to_target / self.lookahead_distance) * (self.target_velocity / max(self.target_velocity, 1))

            # Limit throttle if steering angle is too large
            # if abs(self.steering) >= 0.8:
            #     self.throttle = min(self.throttle, 0.2)
            if distance_to_target < 3.0:
                self.braking = 1 / distance_to_target

            msg.steering = np.clip(self.steering, -1, 1)
            msg.throttle = np.clip(self.throttle, 0, 1)
            msg.braking = np.clip(self.braking, 0, 1)
            msg.header.stamp = self.get_clock().now().to_msg()

            self.get_logger().info(f"Publishing vehicle command: {msg}")
            self.pub_vehicle_cmd.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Failed to compute vehicle command: {e}")

def main(args=None):
    rclpy.init(args=args)
    control = PurePursuitFollowerNode()
    if rclpy.ok():
        rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
