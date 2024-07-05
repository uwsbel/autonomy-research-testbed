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
###############################################################################
## Author: Harry Zhang
###############################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from chrono_ros_interfaces.msg import Body
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

import csv 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # update frequency of this node
        self.freq = 10.0

        # READ IN SHARE DIRECTORY LOCATION
        #package_share_directory = get_package_share_directory('convoy_dART')
        # initialize control inputs
        self.steering_leader = 0.0
        self.throttle_leader = 0.7
        
        self.steering_follower = 0.0
        self.throttle_follower = 0.3
        
        self.braking = 0.0

        # initialize vehicle state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0

        # data that will be used by this class
        self.vehicle_states = []
        self.path = Path()
        self.go = False
        self.vehicle_cmd = VehicleInput()

        
        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.sub_harryInput = self.create_subscription(Twist,'/leader_vehicle/cmd_vel',self.HarryInputs_callback,qos_profile)
        self.sub_odometry = self.create_subscription(Odometry, '/leader_vehicle/odometry/filtered', self.odometry_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '/leader_vehicle/control/vehicle_inputs', 10)
        self.pub_vehicle_state = self.create_publisher(Float64MultiArray, '/leader_vehicle/vehicle_traj', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

    # subscribe manual control inputs
    def HarryInputs_callback(self, msg):
        self.go = True
        self.get_logger().info("received harry's inputs:")
        self.throttle_leader += msg.linear.x
        self.steering_leader += msg.angular.z
        self.get_logger().info("Throttle: %s" % self.throttle_leader)
        self.get_logger().info("Steering: %s" % self.steering_leader)

    # new callback for odometry messages
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #convert quaternion to euler angles
        e0 = msg.pose.pose.orientation.x
        e1 = msg.pose.pose.orientation.y
        e2 = msg.pose.pose.orientation.z
        e3 = msg.pose.pose.orientation.w
        self.theta = np.arctan2(2*(e0*e3+e1*e2),e0**2+e1**2-e2**2-e3**2)
        self.v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
    
    def vehicle_trajectory(self):
        self.vehicle_states.append([self.x, self.y, self.theta, self.v])
        state_array = Float64MultiArray()
        state_array.layout.dim.append(MultiArrayDimension())
        state_array.layout.dim[0].size = 4
        state_array.layout.dim[0].stride = 4 * len(self.vehicle_states)
        state_array.layout.dim[0].label = 'states'
        # Flatten the list of lists to a single list
        flat_list = [item for sublist in self.vehicle_states for item in sublist]
        state_array.data = flat_list
        return state_array
        
    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return
        state_array = self.vehicle_trajectory()
        ### for vehicle one
        msg_leader = VehicleInput()
        msg_leader.steering = np.clip(self.steering_leader, -1.0, 1.0)
        msg_leader.throttle = np.clip(self.throttle_leader, 0, 1)
        
        self.pub_vehicle_cmd.publish(msg_leader)
        # publish vehicle trajectory
        self.pub_vehicle_state.publish(state_array)

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

