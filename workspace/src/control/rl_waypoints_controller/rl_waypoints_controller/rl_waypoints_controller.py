#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
from stable_baselines3 import PPO
import gymnasium as gym

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile


class RLWaypointsController(Node):
    """TODO: Add docstring here"""

    def __init__(self):
        super().__init__("rl_waypoints_controller_node")

        # DEFAULT SETTINGS

        # update frequency of this node
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory("rl_waypoints_controller")

        # ROS PARAMETERS
        self.declare_parameter("control_mode", "RL")
        self.mode = (
            self.get_parameter("control_mode").get_parameter_value().string_value
        )
        self.declare_parameter("control_file", "")
        self.file = (
            self.get_parameter("control_file").get_parameter_value().string_value
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

        # checkpoints dir
        checkpoints_dir = "/home/art/art/workspace/src/control/rl_waypoints_controller/rl_waypoints_controller/network_file"
        self.loaded_model = PPO.load(os.path.join(checkpoints_dir, "ppo_checkpoint"))

        # data that will be used by this class
        self.state = ""
        self.vehicle_cmd = VehicleInput()

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_err_state = self.create_subscription(
            VehicleState, "/vehicle/error_state", self.err_state_callback, qos_profile
        )
        # self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, "~/output/vehicle_inputs", 10
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    # function to process data this class subscribes to
    def err_state_callback(self, msg):
        """Callback for the vehicle state subscriber.

        Read the state of the vehicle from the topic.

        Args:
            msg: The message received from the topic
        """
        # self.get_logger().info("Received '%s'" % msg)
        self.state = msg

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        """Callback for the publisher.

        Publish the vehicle inputs to follow the path. If we are using control inputs from a file, then calculate what the control inputs should be. If the PID controller is being used, multiply the ratio of y/x reference coordinatesby the steering gain, and set constant throttle.
        """
        msg = VehicleInput()
        # TODO: FIX THE PATH PLANNER LINEAR PUBLISHING
        if self.state == "":
            self.get_logger().info("State: " + str(self.state))
            return
        obs_dict = {
            "data": np.array(
                [
                    self.state.pose.position.x,
                    self.state.pose.position.y,
                    self.state.pose.orientation.z,
                    self.state.twist.linear.x,
                ]
            )
        }
        action, _states = self.loaded_model.predict(obs_dict, deterministic=True)
        self.get_logger().info("Action: " + str(action))

        msg.steering = np.clip(0.0, -1, 1)
        msg.throttle = np.clip(0.5, 0, 1)
        msg.braking = np.clip(0.0, 0, 1)

        msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_vehicle_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    control = RLWaypointsController()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
