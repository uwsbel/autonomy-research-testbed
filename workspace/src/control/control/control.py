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

#// =============================================================================
#// Authors: Asher Elmquist, Harry Zhang
#// =============================================================================

import rclpy
import csv 
from rclpy.node import Node
from art_msgs.msg import VehicleState
from chrono_ros_msgs.msg import ChVehicle
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os
import sys
# os.system("python3 template_model.py")
# os.system("python3 template_mpc.py")
###---
##newly added to do mpc project
from casadi import *
from casadi.tools import *
import sys
sys.path.append('/home/art/art/workspace/src/control/control')
#from test_pid_vel_control import speed_control
# from mpc_osqp import mpc_osqp_solver
# from mpc_cvxpy import mpc_cvxpy_solver
#from mpc_cvxpy_v2 import mpc_cvxpy_solver_v2
from mpc_wpts import mpc_wpts_solver
###---



from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # DEFAULT SETTINGS

        #from stefan
        self.first_write = True
        # control node mode
        self.mode = "PID"  # "PID", "File"
        self.file = ""
        self.recorded_inputs = np.array([])
        # update frequency of this node
        self.freq = 50.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory('control')

        # ROS PARAMETERS
        self.declare_parameter('control_mode', 'PID')
        self.mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.declare_parameter('control_file', "")
        self.file = self.get_parameter('control_file').get_parameter_value().string_value

        self.declare_parameter('steering_gain', 1.0)
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.declare_parameter('throttle_gain', 1.0)
        self.throttle_gain = self.get_parameter('throttle_gain').get_parameter_value().double_value

        self.declare_parameter("use_sim_msg", False)
        use_sim_msg = self.get_parameter("use_sim_msg").get_parameter_value().bool_value

        if(self.file == ""):
            self.mode = "PID"
        else:
            file_path = os.path.join(package_share_directory, self.file)
            self.recorded_inputs = np.loadtxt(file_path, delimiter=',')

        self.steering = 0.0
        self.throttle = 0.0 #testing purpose
        self.braking = 0.0

        # data that will be used by this class
        self.state = VehicleState()
        self.groud_truth = ChVehicle()
        self.error_state = ChVehicle()
        self.path = Path()
        if use_sim_msg:
            global VehicleInput
            from chrono_ros_msgs.msg import ChDriverInputs as VehicleInput
        else:
            global VehicleInput
            from art_msgs.msg import VehicleInput
        self.vehicle_cmd = VehicleInput()

        #waits for first path if using PID, otherwise runs right away
        self.go = (self.mode == "File")
        

        # publishers and subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_path = self.create_subscription(Path, '~/input/path', self.path_callback, qos_profile)
        self.sub_err_state = self.create_subscription(ChVehicle, '/vehicle/error_state', self.err_state_callback, qos_profile)
        #self.sub_state = self.create_subscription(ChVehicle, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.sub_state = self.create_subscription(VehicleState, '/vehicle_state', self.state_callback, qos_profile)
        self.sub_ground_truth = self.create_subscription(ChVehicle, '/vehicle/state', self.ground_truth_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '~/output/vehicle_inputs', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        self.g_pos = [-3.15,-1.25]
        self.crl_tspan = 1/10
        self.vel = 0.0

    # function to process data this class subscribes to
    def ground_truth_callback(self, msg):
        #self.get_logger().info("Received '%s'" % msg)
        self.groud_truth = msg

    # function to process data this class subscribes to
    def state_callback(self, msg):
        #self.get_logger().info("Received '%s'" % msg)
        self.state = msg
        self.vel = np.sqrt(self.state.twist.linear.x**2+self.state.twist.linear.y**2)

    def err_state_callback(self,msg):
        self.error_state = msg
        

    def path_callback(self, msg):
        self.go = True
        # self.get_logger().info("Received '%s'" % msg)
        self.path = msg

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return

        msg = VehicleInput()

        if(self.mode == "File"):
            self.calc_inputs_from_file()
        elif(self.mode == "PID" and len(self.path.poses)>0):


            #read the error state:
            e = [self.error_state.pose.position.x, 
                 self.error_state.pose.position.y,
                 self.error_state.pose.orientation.z,
                 self.error_state.twist.linear.x]
            self.get_logger().info(' recieved err state = %s ' % e)
            #read velocity
            velo = self.vel
            #feed in velocity, target point coordinates and current control inputs to the mpc solver
            ### use the mpc solver
            self.throttle, self.steering = mpc_wpts_solver(e,[self.throttle,self.steering],velo,1)
            

            # self.throttle = 0.4
            # self.steering = 0.3
            
            steer_coeff = 1.3
            self.steering = self.steering * steer_coeff
            self.get_logger().info(' control = %s' % [self.throttle, self.steering])

            # if(self.first_write):
            #     os.remove("mpc_0211_efkmpc.csv")
            #     self.first_write = False


            with open ('mpc_square.csv','a', encoding='UTF8') as csvfile:
                my_writer = csv.writer(csvfile)
                #for row in pt:
                my_writer.writerow([self.groud_truth.pose.position.x,self.groud_truth.pose.position.y,self.state.pose.position.x,self.state.pose.position.y,self.throttle,self.steering])
                csvfile.close()

        
        msg.steering = np.clip(self.steering, -1, 1)
        msg.throttle = np.clip(self.throttle, 0, 1)
        msg.braking = np.clip(self.braking, 0, 1)
        self.pub_vehicle_cmd.publish(msg)          
        

    def calc_inputs_from_file(self):
        t = self.get_clock().now().nanoseconds / 1e9 - self.t_start

        self.throttle = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,1])
        self.braking = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,2])
        self.steering = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,3])

        # self.get_logger().info('Inputs %s' % self.recorded_inputs[0,:])

        # self.get_logger().info('Inputs from file: (t=%s, (%s,%s,%s)),' % (t,self.throttle,self.braking,self.steering))

def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
# else:
#     recording_state.tofile('state_record.csv',sep=',',format = '%10.5f')