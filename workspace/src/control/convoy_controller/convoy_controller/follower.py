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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###############################################################################
## Author: Harry Zhang
###############################################################################

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
#from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from art_msgs.msg import VehicleInput
from chrono_ros_interfaces.msg import Body
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

import csv 
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # update frequency of this node
        self.freq = 10.0

        # READ IN SHARE DIRECTORY LOCATION
        #package_share_directory = get_package_share_directory('convoy_dART')
        # initialize control inputs
        self.steering = 0.0
        self.throttle = 0.7

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
        
        self.ref_traj = []
        self.lookahead = 1.0
        
        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub_odometry = self.create_subscription(Odometry, '/follower_vehicle/odometry/filtered', self.odometry_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '/follower_vehicle/driver_inputs', 10)
        self.sub_vehicle_state = self.create_subscription(Odometry, '/leader_vehicle/odometry/filtered', self.trajectory_callback, qos_profile)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        
        # Service to start the node
        self.srv = self.create_service(Trigger, 'start_control', self.start_control_callback)
    
    # subscribe manual control inputs
    def trajectory_callback(self, msg):
        self.go = True
        # Extract data from the Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = np.arctan2(2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
                           1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y + msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))
        v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
        
        self.ref_traj.append([x, y, theta, v])

        # Log or process the array
        self.get_logger().info(f'Received trajectory point: (x: {x}, y: {y}, theta: {theta}, v: {v})')

    # function to process data this class subscribes to
    def odometry_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Convert quaternion to Euler angles
        e0 = msg.pose.pose.orientation.x
        e1 = msg.pose.pose.orientation.y
        e2 = msg.pose.pose.orientation.z
        e3 = msg.pose.pose.orientation.w
        self.theta = np.arctan2(2 * (e0 * e3 + e1 * e2), e0**2 + e1**2 - e2**2 - e3**2)
        self.v = np.sqrt(msg.twist.twist.linear.x ** 2 + msg.twist.twist.linear.y ** 2)
        
    def error_state(self):
        x_current = self.x
        y_current = self.y
        theta_current = self.theta
        v_current = self.v
        
        # Post process theta
        while theta_current < -np.pi:
            theta_current = theta_current + 2 * np.pi
        while theta_current > np.pi:
            theta_current = theta_current - 2 * np.pi

        dist = np.zeros((1, len(self.ref_traj)))
        for i in range(len(self.ref_traj)):
            dist[0][i] = (x_current + np.cos(theta_current) * self.lookahead - self.ref_traj[i][0]) ** 2 + (y_current + np.sin(theta_current) * self.lookahead - self.ref_traj[i][1]) ** 2
        index = dist.argmin()

        ref_state_current = self.ref_traj[index]
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

        RotM = np.array([ 
            [np.cos(-theta_current), -np.sin(-theta_current)],
            [np.sin(-theta_current), np.cos(-theta_current)]
        ])

        errM = np.array([[ref_state_current[0] - x_current], [ref_state_current[1] - y_current]])

        errRM = RotM @ errM

        error_state = [errRM[0][0], errRM[1][0], err_theta, ref_state_current[3] - v_current]

        return error_state        

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if not self.go:
            return
        e = self.error_state()
        steering = sum([x * y for x, y in zip(e, [0.02176878, 0.72672704, 0.78409284, -0.0105355])])
        self.throttle = 0.7
        
        position_error = np.sqrt(e[0] ** 2 + e[1] ** 2)
        self.throttle = np.clip(0.5 * position_error, 0, 1)
        #velocity_error = e[3]  # The velocity error is the fourth element in the error state
        #self.throttle = np.clip(self.throttle + throttle_gain * velocity_error, 0, 1)

        # Ensure steering can't change too much between timesteps, smooth transition
        delta_steering = steering - self.steering
        if abs(delta_steering) > 0.25:
            self.steering = self.steering + 0.25 * delta_steering / abs(delta_steering)
            self.get_logger().info("steering changed too much, smoothing")
        else:
            self.steering = steering
        
        ### for vehicle one
        msg_follower = VehicleInput()
        msg_follower.steering = np.clip(self.steering, -1.0, 1.0)
        msg_follower.throttle = np.clip(self.throttle, 0, 1)
        self.pub_vehicle_cmd.publish(msg_follower)
        
        # Record the follower vehicle data, lead vehicle trajectory
        np.savetxt('leader_vehicle_traj.csv', self.ref_traj, delimiter=',')
        with open('follower_vehicle_traj.csv', 'a', encoding='UTF8') as csvfile:
            my_writer = csv.writer(csvfile, quoting=csv.QUOTE_NONE, escapechar=' ')
            my_writer.writerow([self.x, self.y, self.theta, self.v])
            csvfile.close()
    
    def start_control_callback(self, request, response):
        self.go = True
        response.success = True
        response.message = "Control node started"
        self.get_logger().info("Received start signal")
        return response

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(control, executor)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

