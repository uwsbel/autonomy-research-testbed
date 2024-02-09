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
from art_perception_msgs.msg import ObjectArray, Object
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import interp1d, splev, splprep
import os
import json


class WaypointsPathPlannerNode(Node):
    """A path planning node based on reading waypoints from a csv file.

    This path planner reads a desired path from a csv file. It then subscribes to the current state of the vehicle, and computes an error state between the desired path and the current state. The path is defined by coordinates, as well as a desired speed and heading for each waypoint defined.

    Attributes:
        lookahead: How far ahead of the car the target waypoint should be.
        state: The current state of the vehicle.
        error_state: The error between the vehicles current state and the target state.


    """

    def __init__(self):
        """initialize the Path planning Node.

        Initialize the path planning node, and set up the publishers / subscribers.

        """
        super().__init__("waypoints_path_planner_node")

        # update frequency of this node
        self.freq = 10.0

        self.file = open(
            "/home/art/art/workspace/src/path_planning/waypoints_path_planner/waypoints_path_planner/path.csv"
        )
        self.ref_traj = np.loadtxt(self.file, delimiter=",")
        self.final_x = self.ref_traj[-1, 0]
        self.final_y = self.ref_traj[-1, 1]

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory("waypoints_path_planner")

        self.declare_parameter("lookahead", 2.0)
        self.lookahead = (
            self.get_parameter("lookahead").get_parameter_value().double_value
        )

        # data that will be used by this class
        self.state = VehicleState()

        # subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_state = self.create_subscription(
            VehicleState, "~/input/vehicle_state", self.state_callback, qos_profile
        )

        # publishers
        self.pub_err_state = self.create_publisher(
            VehicleState, "~/output/error_state", 10
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    # function to process data this class subscribes to
    def state_callback(self, msg):
        """The state Callback.

        The callback to read the current state of the vehicle.

        Args:
            msg: The message received from the vehicle state topic

        """
        # self.get_logger().info("Received '%s'" % msg)
        self.state = msg

    def wpts_path_plan(self):
        """The Waypoints path planner.

        Computes the error state between the target state and the current state and the target state. First, get the target point which is closest ot the current state (plus the lookahead distance in the x direction). Then, get the error state by subtracting the current state from the target state (modular subtraction for the heading).

        Returns:
             The error state, and the target reference waypoint we are currently going to.

        """
        x_current = self.state.pose.position.x
        y_current = self.state.pose.position.y
        # theta_current = np.arctan2( 2*(x*w+z*y), x**2-w**2+y**2-z**2 )
        theta_current = self.state.pose.orientation.z - 0.24845347641462115
        while theta_current < -np.pi:
            theta_current = theta_current + 2 * np.pi
        while theta_current > np.pi:
            theta_current = theta_current - 2 * np.pi

        v_current = np.sqrt(
            self.state.twist.linear.x**2 + self.state.twist.linear.y**2
        )

        # self.get_logger().info("SPEED: "+str(v_current))

        dist = np.zeros((1, len(self.ref_traj[:, 1])))
        for i in range(len(self.ref_traj[:, 1])):
            dist[0][i] = dist[0][i] = (
                x_current + np.cos(theta_current) * self.lookahead - self.ref_traj[i][0]
            ) ** 2 + (
                y_current + np.sin(theta_current) * self.lookahead - self.ref_traj[i][1]
            ) ** 2
            index = dist.argmin()

        ref_state_current = list(self.ref_traj[index, :])
        err_theta = 0
        ref = ref_state_current[2]
        act = theta_current

        if (ref > 0 and act > 0) or (ref <= 0 and act <= 0):
            err_theta = ref - act
        elif ref <= 0 and act > 0:
            if abs(ref - act) < abs(2 * np.pi + ref - act):
                err_theta = -abs(act - ref)
            else:
                err_theta = abs(2 * np.pi + ref - act)
        else:
            if abs(ref - act) < abs(2 * np.pi - ref + act):
                err_theta = abs(act - ref)
            else:
                err_theta = -abs(2 * np.pi - ref + act)

        RotM = np.array(
            [
                [np.cos(-theta_current), -np.sin(-theta_current)],
                [np.sin(-theta_current), np.cos(-theta_current)],
            ]
        )

        errM = np.array(
            [[ref_state_current[0] - x_current], [ref_state_current[1] - y_current]]
        )

        errRM = RotM @ errM

        error_state = [
            errRM[0][0],
            errRM[1][0],
            err_theta,
            ref_state_current[3] - v_current,
        ]

        return error_state, ref_state_current[3]

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        """The Publisher.

        Calls the waypoint path planner, and publishes the current error state.

        """
        msg = VehicleState()
        error_state, ref_vel = self.wpts_path_plan()

        msg.pose.position.x = error_state[0]
        msg.pose.position.y = error_state[1]
        msg.pose.orientation.z = error_state[2]
        msg.twist.linear.x = error_state[3]
        msg.twist.linear.y = ref_vel
        self.pub_err_state.publish(msg)


def main(args=None):
    # print("=== Starting Path Planning Node ===")
    rclpy.init(args=args)
    planner = WaypointsPathPlannerNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
