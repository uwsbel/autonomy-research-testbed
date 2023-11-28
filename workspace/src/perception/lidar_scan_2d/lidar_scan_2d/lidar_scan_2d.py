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
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import interp1d, splev, splprep
import os
import json
import math


class lidar_scan_2d(Node):
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
        super().__init__("lidar_scan_2d_node")

        # update frequency of this node
        self.freq = 10.0

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory("lidar_scan_2d")

        # subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_state = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile
        )

        # publishers
        self.pub_scan = self.create_publisher(Float32MultiArray, "/reduced_scan", 10)
        self.reduced_lidar_data = []
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    def lidar_callback(self, msg):
        self.go = True
        # self.get_logger().info("received lidar data")
        self.lidar_data = msg
        self.raw_lidar_data = msg.ranges
        # self.get_logger().info("original size: %s" % len(self.raw_lidar_data))
        self.reduced_lidar_data = self.reduce_lidar()

    def reduce_lidar(self):
        # Step 1: Flip the first and fourth quarters and append them together
        mid_index = len(self.raw_lidar_data) // 2
        first_quarter = self.raw_lidar_data[:mid_index]
        fourth_quarter = self.raw_lidar_data[mid_index:]
        flipped_first_quarter = first_quarter[::-1]
        flipped_fourth_quarter = fourth_quarter[::-1]
        filtered_list = flipped_first_quarter + flipped_fourth_quarter

        # Step 2: Replace ".inf" with 30.0
        filtered_list = [30.0 if math.isinf(val) else val for val in filtered_list]

        # Step 3: Divide the list into 18 parts and grab the minimum value for each part
        num_parts = 18
        part_size = len(filtered_list) // num_parts
        min_values = [
            min(filtered_list[i : i + part_size])
            for i in range(0, len(filtered_list), part_size)
        ]
        return min_values

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        """The Publisher.

        Calls the waypoint path planner, and publishes the current error state.

        """
        msg = Float32MultiArray()

        msg.data = self.reduced_lidar_data

        self.pub_scan.publish(msg)


def main(args=None):
    # print("=== Starting Path Planning Node ===")
    rclpy.init(args=args)
    scan = lidar_scan_2d()
    rclpy.spin(scan)
    scan.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
