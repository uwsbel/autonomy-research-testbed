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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import pydof18 as d18
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
#from pid import pidControl

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

from keras_core.models import load_model

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # DEFAULT SETTINGS


        self.recorded_inputs = np.array([])
        # update frequency of this node
        self.freq = 10.0

        self.t_start = self.get_clock().now().nanoseconds / 1e9

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory('control')

        # ROS PARAMETERS
        self.declare_parameter('control_mode', 'PID')# "PID", "File", "MPC", "NN", "MC"
        self.mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.declare_parameter('control_file', "")
        self.file = self.get_parameter('control_file').get_parameter_value().string_value

        self.declare_parameter('steering_gain', 1.0)
        self.steering_gain = self.get_parameter('steering_gain').get_parameter_value().double_value
        self.declare_parameter('throttle_gain', 1.0)
        self.throttle_gain = self.get_parameter('throttle_gain').get_parameter_value().double_value

        self.declare_parameter("use_sim_msg", False)
        use_sim_msg = self.get_parameter("use_sim_msg").get_parameter_value().bool_value


        ## read control inputs file
        # self.input_file = open("/home/art/art/workspace/src/control/control/r1.txt")
        # self.input_list = np.loadtxt(self.input_file, delimiter=" ")
        
        if(self.mode == "File"):
            file_path = os.path.join(package_share_directory, self.file)
            self.recorded_inputs = np.loadtxt(file_path, delimiter=',')
     
        self.steering = 0.0
        self.throttle = 0.7 #testing purpose
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
        self.sub_err_state = self.create_subscription(ChVehicle, '~/input/error_state', self.err_state_callback, qos_profile)
        #self.sub_state = self.create_subscription(ChVehicle, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        self.sub_ground_truth = self.create_subscription(ChVehicle, '~/input/true_state', self.ground_truth_callback, qos_profile)
        if(self.mode == 'MC'):
            self.sub_harryInput = self.create_subscription(Twist,'/cmd_vel',self.HarryInputs_callback,qos_profile)
        self.pub_vehicle_cmd = self.create_publisher(VehicleInput, '~/output/vehicle_inputs', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        self.vel = 0.0
        self.heading = 0.0
        
        ##### For ML stuffs
        if(self.mode == "NN"):
            self.model_mc = load_model('/home/art/art/workspace/src/control/control/keras_ml.keras')
            self.get_logger().info('NN model loaded')


        self.init_vehicle()


        self.Kp = 0.1  
        self.Kd = 0.01  
        self.Ki = 0.0  

        #self.solver = None
        self.t = 0
        

        self.csv_file = open('velocity_error.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Velocity Error'])


    # function to process data this class subscribes to
    def ground_truth_callback(self, msg):
        self.groud_truth = msg

    # function to process data this class subscribes to
    def state_callback(self, msg):
        self.state = msg
        #TODO: We need to fix the -0.248
        self.heading =msg.pose.orientation.z-0.24845347641462115
        while self.heading<-np.pi:
            self.heading = self.heading+2*np.pi
        while self.heading>np.pi:
            self.heading = self.heading - 2*np.pi
        self.vel = np.sqrt(self.state.twist.linear.x**2+self.state.twist.linear.y**2)

    def err_state_callback(self,msg):
        self.go = True
        self.error_state = msg
        

    def path_callback(self, msg):
        self.path = msg

    def HarryInputs_callback(self,msg):
        self.get_logger().info("received harry's inputs: %s" % msg)
        self.throttle += msg.linear.x
        self.steering += msg.angular.z

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return

        msg = VehicleInput()

        if(self.mode == "File"):
            self.calc_inputs_from_file()
        elif(len(str(self.error_state.pose.position.x))>0):
            ##read the error state:
            e = [self.error_state.pose.position.x, 
                 self.error_state.pose.position.y,
                 self.error_state.pose.orientation.z,
                 self.error_state.twist.linear.x]
            ref_vel = self.error_state.twist.linear.y


            

            #self.get_logger().info(' recieved err state = %s ' % e)
            if(self.mode == 'MC'):
                if e[3]>0.3:
                    self.get_logger().info('-----------------FASTER!!!!!!!------------------')
                elif e[3]<-0.2:
                    self.get_logger().info('-----------------SLOW DOWN!!!!!!!------------------')
                else:
                    self.get_logger().info('-----------------GOOD!!!!!!!------------------')
            
                
            #read velocity
            velo = self.vel
            #feed in velocity, target point coordinates and current control inputs to the mpc solver
            # #MPC method
            if(self.mode == 'MPC'):
                self.throttle, self.steering = mpc_wpts_solver(e,[self.throttle,self.steering],velo,ref_vel)

            
            # # #ML method
            ###learning mpc
            # self.throttle = sum([x * y for x, y in zip(e, [ 0.42747883,-0.10800391,0.06556592,1.2141007])])
            # self.steering = sum([x * y for x, y in zip(e, [0.02855189,  1.21156572,  0.69078731, 0.09465685])])
            # ##learning manual
            if(self.mode == 'PID'):
                #self.throttle = sum([x * y for x, y in zip(e, [0.90195976 ,-0.00169086 , 0.10097878 , 0.0058228 ])])
                #self.steering = sum([x * y for x, y in zip(e, [ 0.09133028 , 0.3940351 ,  0.13070601, -0.12223042])])
                self.throttle = sum([x * y for x, y in zip(e, [ 0.42747883,-0.10800391,0.06556592,1.2141007])])
                self.steering = sum([x * y for x, y in zip(e, [0.02855189,  1.21156572,  0.69078731, 0.09465685])])
            
            ## apply black box driving
            if(self.mode == 'NN'):
                err = np.array(e).reshape(1,-1)
                self.get_logger().info(str(err))
                self.get_logger().info()
                ctrl = self.model_mc.predict(err)
                self.get_logger().info(str(ctrl))
                self.throttle = ctrl[0,0]
                #self.get_logger().info('NN throttle =' +str (self.throttle))
                self.steering = ctrl[0,1]
                #self.get_logger().info('NN steering = ' +str(self.steering))

            if(not self.mode == 'MC'):
                self.steering = self.steering * 1.6
            self.throttle = 1.0
            self.steering = 0.3

            throttle = self.throttle
            max_iterations = 10
            iteration = 0

    
            error_sum = 0
            error_diff = 0
            error_prev = 0

            
            steering = self.steering
            
            while iteration < max_iterations:
                
                # Call the model with the current throttle and steering
                solverUpdated = self.dof18_model(self.solver, self.t , throttle, steering)

                # Extract the velocity from the solver
                dof18_vel = solverUpdated.m_veh_state._u

                # Calculate the error (for now just set refernce to 0.2)
                error = ref_vel - dof18_vel

                # Call the PID controller
                throttle = self.pid_control(error, error_sum, error_diff, error_prev)

                if(error < 0.01):
                    break

                iteration += 1
            self.t = self.t + 0.1

            self.throttle = throttle
            
            # Finally update the global solver states
            self.solver.m_veh_state = d18.VehicleState(solverUpdated.m_veh_state)
            self.solver.m_tirelf_st = d18.TMeasyState(solverUpdated.m_tirelf_state)
            self.solver.m_tirerf_st = d18.TMeasyState(solverUpdated.m_tirerf_state)
            self.solver.m_tirelr_st = d18.TMeasyState(solverUpdated.m_tirelr_state)
            self.solver.m_tirerr_st = d18.TMeasyState(solverUpdated.m_tirerr_state)
            
            
            with open('velocity_error.csv', 'a', newline='') as velocity_error_csv:
                velocity_error_writer = csv.writer(velocity_error_csv)
                velocity_error_writer.writerow([error, dof18_vel, ref_vel, self.throttle, self.t])
            
            with open ('circle_sim_testing.csv','a', encoding='UTF8') as csvfile:
                my_writer = csv.writer(csvfile)
                #for row in pt:
                my_writer.writerow([self.groud_truth.twist.linear.x,self.groud_truth.twist.linear.y,error,self.throttle,self.steering])
                csvfile.close()

        
        msg.steering = np.clip(self.steering, -1, 1)
        #msg.throttle = np.clip(self.throttle, 0, 1)
        msg.throttle =  self.throttle
        msg.braking = np.clip(self.braking, 0, 1)
        self.pub_vehicle_cmd.publish(msg)   

    # def pid_control(self, error):
    #     # Calculate PID control output
    #     self.error_sum += error
    #     self.error_diff = error - self.error_prev
    #     self.error_prev = error

    #     throttle = self.throttle + self.Kp * error + self.Ki * self.error_sum + self.Kd * self.error_diff

    #     throttle = np.clip(throttle, 0, 1)

    #     return throttle     

    def init_vehicle(self):

        vehParamsJsonPath = "/home/art/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/json/ART/vehicle.json"
        tireParamsJsonPath = "/home/art/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/json/ART/tmeasy.json"
        driver_file = "/home/art/low-fidelity-dynamic-models/wheeled_vehicle_models/18dof/data/input/acc.txt"
        solver = d18.d18SolverHalfImplicit()
        solver.Construct(vehParamsJsonPath, tireParamsJsonPath, driver_file)

        # Set time step
        solver.SetTimeStep(1e-3)

        # Initialize solver (set initial conditions)
        veh_st = d18.VehicleState()
        tirelf_st = d18.TMeasyState()
        tirerf_st = d18.TMeasyState()
        tirelr_st = d18.TMeasyState()
        tirerr_st = d18.TMeasyState()
        solver.Initialize(veh_st, tirelf_st, tirerf_st, tirelr_st, tirerr_st)

        self.solver = solver

    # def dof18_model(self, t, throttle, steering):
    #     required_time = t + 0.1    
    #     new_time = t
    #     while(new_time < required_time):
    #         new_time = self.solver.IntegrateStep(t, throttle, steering, 0)
    #         t = new_time

    #     return self.solver


    def dof18_model(self,solverOld, t, throttle, steering):
        
        required_time = t + 0.1

        # Create a new solver object
        solverNew = d18.d18SolverHalfImplicit()

        # copy over solver parameters that were there at the start of the iterations (ideally also the same as the initial parameters)
        solverNew.m_veh_param = d18.VehicleParam(solverOld.m_veh_param)
        solverNew.m_tire_param = d18.TMeasyParam(solverOld.m_tire_param)

        # Set the simulation time step
        solverNew.SetTimeStep(1e-3)

        # Copy over the solver states that were there at the start of the iterations
        solverNew.m_veh_state = d18.VehicleState(solverOld.m_veh_state)
        solverNew.m_tirelf_st = d18.TMeasyState(solverOld.m_tirelf_state)
        solverNew.m_tirerf_st = d18.TMeasyState(solverOld.m_tirerf_state)
        solverNew.m_tirelr_st = d18.TMeasyState(solverOld.m_tirelr_state)
        solverNew.m_tirerr_st = d18.TMeasyState(solverOld.m_tirerr_state)
        
        
        i = 0
        # Integrate till the required time
        while t < required_time:
            # Integrate till the next time step
            new_time = solverNew.IntegrateStep(t, throttle, steering, 0)
            t = new_time  # new_time is where the solver is right now
            
            
            
        # Once we have intgrated till the required time, we can return the solver object
        
        
        return solverNew


    def pid_control(self,error, error_sum, error_diff, error_prev):
        # Working PID parameters
        Kp = 0.1
        Kd = 0.08
        Ki = 25.0

        # do some PID

        error_sum += error
        error_diff = error - error_prev
        error_prev = error

        throttle = Kp*error + Ki*error_sum + Kd*error_diff

        throttle = np.clip(throttle, 0, 1)

        return throttle

          
        

    def calc_inputs_from_file(self):

        self.throttle = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,1])
        self.braking = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,2])
        self.steering = np.interp(t,self.recorded_inputs[:,0],self.recorded_inputs[:,3])


def main(args=None):
    rclpy.init(args=args)
    control = ControlNode()
    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
