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
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
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

ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
sys.path.insert(0, os.path.abspath(ament_tools_root))

#from test_pid_vel_control import speed_control
# from mpc_osqp import mpc_osqp_solver
# from mpc_cvxpy import mpc_cvxpy_solver
#from mpc_cvxpy_v2 import mpc_cvxpy_solver_v2
from mpc_wpts import mpc_wpts_solver
###---



from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from keras_core.models import load_model

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
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9
        self.stop = False
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

        ## read control inputs file
        #self.input_file = open("/home/art-ishaan/art-ishaan/workspace/src/control/control/r1_10hz.txt")
        #self.input_list = np.loadtxt(self.input_file, delimiter=" ")
        self.index = 0
        #self.endind = np.shape(self.input_list)[0]
        self.steering = 0.0
        self.throttle = 0.0 #testing purpose
        self.braking = 0.0
        self.vel = 0.0
        # data that will be used by this class
        self.state = VehicleState()
        self.groud_truth = ChVehicle()
        self.error_state = VehicleState()
        self.mocap_state = Pose2D()
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
        self.sub_err_state = self.create_subscription(VehicleState, '/vehicle/error_state', self.err_state_callback, qos_profile)
        #self.sub_state = self.create_subscription(ChVehicle, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.sub_state = self.create_subscription(VehicleState, '/vehicle_state', self.state_callback, qos_profile)
        self.sub_ground_truth = self.create_subscription(ChVehicle, '/vehicle/state', self.ground_truth_callback, qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '~/output/vehicle_inputs', 10)
        self.sub_mocap = self.create_subscription(Pose2D,'/ART1/ground_pose',self.mocap_callback,qos_profile)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        

        self.g_pos = [-3.15, -1.25]
        self.crl_tspan = 1/10
        self.vel = 0.0
        self.heading = 0.0
        ## manual control single speed
        self.model_mc =load_model('/home/art-ishaan/art-ishaan/workspace/src/control/control/keras_ml_learnMC_1la_0808.keras') 
        ## manual control multispeeds
        #self.model_mc =load_model('/home/art-ishaan/art-ishaan/workspace/src/control/control/keras_ml_learnMC_1la_0828.keras') 
        ## MPC single speed
        self.model_mpc =load_model('/home/art-ishaan/art-ishaan/workspace/src/control/control/keras_ml_learnMPC_1la_0815.keras') 
        ## MPC multispeed
        #self.model_mpc =load_model('/home/art-ishaan/art-ishaan/workspace/src/control/control/keras_ml_learnMPC_1la_0828.keras') 
    # function to process data this class subscribes to
    def ground_truth_callback(self, msg):
        #self.get_logger().info("Received '%s'" % msg)
        self.groud_truth = msg
    #mocap callback
    def mocap_callback(self,msg):
        self.mocap_state = msg
    # function to process data this class subscribes to
    def state_callback(self, msg):
        #self.get_logger().info("Received '%s'" % msg)
        self.state = msg
        ## velocity from EKF
        #self.vel = np.sqrt(self.state.twist.linear.x**2+self.state.twist.linear.y**2)
        ## velocity from dt of RTKGPS position
        self.vel = self.state.twist.linear.z
        
    def err_state_callback(self,msg):
        self.error_state = msg
        

    def path_callback(self, msg):
        self.go = True
        # self.get_logger().info("Received '%s'" % msg)
        self.path = msg

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        #if(not self.go):
        #    return

        msg = VehicleInput()


        # ########################################################mpc work########################################################
        #read the error state:
        e = [self.error_state.pose.position.x, 
                self.error_state.pose.position.y,
                self.error_state.pose.orientation.z,
                self.error_state.twist.linear.x]
        ref_vel = self.error_state.twist.linear.y
        #self.get_logger().info(' error_state = %s' % e)
        #self.get_logger().info(' recieved reference velocity = %s ' % ref_vel)
        # if(e[0] == 0 and e[1] == 0 and e[2] == 0 and e[3] == 0):
        #     self.stop = True
        #self.get_logger().info(' recieved err state = %s ' % e)
        
        #read velocity
        #self.get_logger().info('Velocity is: '+ str(self.vel))
        
        #feed in velocity, target point coordinates and current control inputs to the mpc solver
        ### use the mpc solver
        #self.throttle, self.steering = mpc_wpts_solver(e,[self.throttle,self.steering],velo,ref_vel)
        
        
        ###learning mpc 0 la
        # self.throttle = sum([x * y for x, y in zip(e, [ 0.42747883,-0.10800391,0.06556592,1.2141007])])
        # self.steering = sum([x * y for x, y in zip(e, [0.02855189,  1.21156572,  0.69078731, 0.09465685])])
        # # ##learning mpc 1 la
        #self.throttle = sum([x * y for x, y in zip(e, [0.37013526 ,0.00507144, 0.15476554 ,1.0235402 ])])
        #self.steering = sum([x * y for x, y in zip(e, [0.02176878 , 0.72672704 , 0.78409284 ,-0.0105355 ])])
        
        
        # ##learning manual 0 la
        # self.throttle = sum([x * y for x, y in zip(e, [ 0.01143464 , 0.00701342 , 0.01340604 ,-0.10267845])]) + 0.85
        # self.steering = sum([x * y for x, y in zip(e, [-0.04234881 , 0.26478591 , 0.78722048 ,-0.04502433])])
        ##learning manual 1 la
        #self.throttle = sum([x * y for x, y in zip(e, [ 0.85433594 , 0.0537554  , 0.16293445 ,-0.1419279 ])])
        #self.steering = sum([x * y for x, y in zip(e, [ 0.01908044 , 0.36068382 , 0.60325371 ,-0.17265345])])
        
        ### hardcode control
        #self.throttle = 1.0
        #self.steering = 0.0
        

        #Keras control, not clear if need steer_coeff....
        err = np.array(e).reshape(1,-1)
        #ctrl = self.model_mpc.predict(err)
        ctrl = self.model_mc.predict(err)
        self.throttle = ctrl[0,0]
        self.steering = ctrl[0,1]*1.5 
        #self.steering = ctrl[0,1]
        # steer_coeff = 0.7
        # self.steering = self.steering * steer_coeff
        
        





        self.get_logger().info(' control = %s' % [self.throttle, self.steering])
        #self.get_logger().info('time at = %s' % self.input_list[self.index,0])
        
        
        # with open ('control_pos_log.csv','a', encoding='UTF8') as csvfile:
        #     my_writer = csv.writer(csvfile)
        #     #for row in pt:
        #     my_writer.writerow([self.input_list[self.index,0],self.mocap_state.x,self.mocap_state.y,self.mocap_state.theta,self.throttle,self.steering])
        #     csvfile.close()


        
        msg.steering = np.clip(self.steering, -1, 1)
        msg.throttle = np.clip(self.throttle, 0.0, 1)
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
