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

# // =============================================================================
# // Authors: Asher Elmquist, Harry Zhang
# // =============================================================================

import rclpy
import csv
from rclpy.node import Node
from art_msgs.msg import VehicleState
from geometry_msgs.msg import PoseStamped
from art_msgs.msg import VehicleInput
from geometry_msgs.msg import Twist

import numpy as np
import os
import sys
import torch

os.environ["KERAS_BACKEND"] = "torch"

sys.path.append(
    "/home/art/art/workspace/src/control/neural_network_controller/neural_network_controller"
)

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

from keras_core.models import load_model


class NeuralNetworkControllerNode(Node):
    def __init__(self):
        super().__init__("neural_network_controller_node")

        # DEFAULT SETTINGS

        # update frequency of this node
        self.freq = 10.0

        # ROS PARAMETERS
        self.declare_parameter("use_sim_msg", False)
        use_sim_msg = self.get_parameter("use_sim_msg").get_parameter_value().bool_value

        self.steering = 0.0
        self.throttle = 0.7  # testing purpose
        self.braking = 0.0

        # data that will be used by this class
        self.state = VehicleState()
        self.error_state = VehicleState()
        self.vehicle_cmd = VehicleInput()

        # waits for first path if using PID, otherwise runs right away
        self.go = False

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_err_state = self.create_subscription(
            VehicleState, "~/input/error_state", self.err_state_callback, qos_profile
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleInput, "~/output/vehicle_inputs", 10
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)
        self.vel = 0.0
        self.heading = 0.0

        self.model_mc = load_model(
            "/home/art/art/workspace/src/control/neural_network_controller/neural_network_controller/keras_ml.keras"
        )

    # function to process data this class subscribes to
    def state_callback(self, msg):
        self.state = msg
        # TODO: We need to fix the -0.248
        self.heading = msg.pose.orientation.z - 0.24845347641462115
        while self.heading < -np.pi:
            self.heading = self.heading + 2 * np.pi
        while self.heading > np.pi:
            self.heading = self.heading - 2 * np.pi
        self.vel = np.sqrt(
            self.state.twist.linear.x**2 + self.state.twist.linear.y**2
        )

    def err_state_callback(self, msg):
        self.go = True
        self.error_state = msg

    def HarryInputs_callback(self, msg):
        self.get_logger().info("received harry's inputs: %s" % msg)
        self.throttle += msg.linear.x
        self.steering += msg.angular.z

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if not self.go:
            return

        msg = VehicleInput()

        ##read the error state:
        e = [
            self.error_state.pose.position.x,
            self.error_state.pose.position.y,
            self.error_state.pose.orientation.z,
            self.error_state.twist.linear.x,
        ]
        ref_vel = self.error_state.twist.linear.y

        err = np.array(e).reshape(1, -1)
        self.get_logger().info(str(err))

        ctrl = self.model_mc.predict(err)
        self.throttle = ctrl[0, 0]
        self.steering = ctrl[0, 1]

        self.steering = self.steering * 1.6

        msg.steering = np.clip(self.steering, -1, 1)
        msg.throttle = np.clip(self.throttle, 0, 1)
        msg.braking = np.clip(self.braking, 0, 1)
        self.pub_vehicle_cmd.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    control = NeuralNetworkControllerNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
