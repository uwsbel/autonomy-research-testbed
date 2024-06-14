import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

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

        self.steering = 0.0
        self.throttle = 0.0
        self.braking = 0.0

        # data that will be used by this class
        self.state = ""
        self.target_state = VehicleState()
        self.current_state = VehicleState()
        self.vehicle_cmd = VehicleInput()

        # waits for first state data
        self.go = False

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_target_state = self.create_subscription(
            VehicleState, "/artcar_1/vehicle/filtered_state", self.target_state_callback, qos_profile
        )
        self.sub_current_state = self.create_subscription(
            VehicleState, "/artcar_2/vehicle/filtered_state", self.current_state_callback, qos_profile
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, "/artcar_2/input/driver_inputs", 10
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    def target_state_callback(self, msg):
        """Callback for the target vehicle state subscriber.

        Read the state of the target vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.go = True
        self.target_state = msg
        self.get_logger().info(f"Received target state: {msg}")

    def current_state_callback(self, msg):
        """Callback for the current vehicle state subscriber.

        Read the state of the current vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        self.current_state = msg
        self.get_logger().info(f"Received current state: {msg}")

    def pub_callback(self):
        """Callback for the publisher.

        Publish the vehicle inputs to follow the target vehicle.
        """

        msg = VehicleInput()

        # Calculate control inputs based on the state data
        target_position = self.target_state.pose.position
        current_position = self.current_state.pose.position

        dx = target_position.x - current_position.x
        dy = target_position.y - current_position.y

        ratio = dy / dx if dx != 0 else 0.0
        steering = self.steering_gain * ratio
        # ensure steering can't change too much between timesteps, smooth transition
        delta_steering = steering - self.steering
        if abs(delta_steering) > 0.1:
            self.steering = self.steering + 0.1 * delta_steering / abs(delta_steering)
        else:
            self.steering = steering

        distance = np.sqrt(dx ** 2 + dy ** 2)
        self.throttle = self.throttle_gain * distance / 10.0  # proportional throttle

        msg.steering = np.clip(self.steering, -1, 1)
        msg.throttle = np.clip(self.throttle, 0, 1)
        msg.braking = np.clip(self.braking, 0, 1)

        msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Publishing vehicle command: {msg}")
        self.get_logger().info(f"Current state: steering={msg.steering}, throttle={msg.throttle}, braking={msg.braking}")
        self.pub_vehicle_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    control = PIDLateralFollowerNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
