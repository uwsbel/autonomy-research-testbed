import csv
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from chrono_ros_msgs.msg import ChVehicle
from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt
import matplotlib
import math
import numpy as np


import sys
import os


ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
sys.path.insert(0, os.path.abspath(ament_tools_root))

from EKF import EKF
from particle_filter import particle_filter as PF
from chrono_coordinate_transfer import graph

class StateEstimationNode(Node):
    def __init__(self):
        super().__init__('state_estimation_node')
        

        # ROS PARAMETERS
        self.use_sim_msg = self.get_parameter("use_sim_time").get_parameter_value().bool_value

        self.declare_parameter('SE_mode', "GT")
        self.SE_mode = self.get_parameter('SE_mode').get_parameter_value().string_value


        #update frequency of this node
        self.freq = 10.0

        self.gps = ""
        self.groundTruth = ""
        self.mag = ""

        if self.use_sim_msg:
            global VehicleInput
            from chrono_ros_msgs.msg import ChDriverInputs as VehicleInput
        else:
            global VehicleInput
            from art_msgs.msg import VehicleInput



        #x, y, from measurements
        self.x = 0
        self.y = 0
        
        #what we will be using for our state vector. (x, y, theta yaw, v vel)
        self.state = np.zeros((4,1))
        np.vstack(self.state)

        
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_theta = 0.0
        self.init_v = 0.0
        self.state[0,0] = self.init_x
        self.state[1,0] = self.init_y
        self.state[2,0] = self.init_theta
        self.state[3,0] = self.init_v

        self.gps_ready = False

        #true X, True Y, velocity, z, heading(degrees)
        self.gtvy = 0
        self.gty = 0
        self.gtvx = 0
        self.gtx = 0
        self.gtz = 0
        self.D = 0

        #origin, and whether or not the origin has been set yet.
        self.origin_set = False
        self.orig_heading_set = False


        #inputs to the vehicle
        self.throttle = 0.0
        self.steering = 0

        #time between imu updates, sec
        self.dt_gps = 0.1
        
        #filter
        if(self.SE_mode == "EKF"):
            self.ekf = EKF(self.dt_gps)
        elif(self.SE_mode == "PF"):
            self.pf = PF(self.dt_gps)

        #our graph object, for reference frame
        self.graph =  graph()

        #subscribers
        self.sub_gps = self.create_subscription(NavSatFix, '~/input/gps', self.gps_callback, 1)
        if(self.use_sim_msg):
            self.sub_groud_truth = self.create_subscription(ChVehicle, '~/input/groundTruth', self.ground_truth_callback, 1)

        self.sub_mag = self.create_subscription(MagneticField, "~/input/magnetometer", self.mag_callback, 1)
        #self.sub_gyro = self.create_subscription(Imu, "~/input/gyro", self.gyro_callback, 10)
        #self.sub_accel = self.create_subscription(Imu, "~/input/accel", self.accel_callback, 100)
        self.sub_control = self.create_subscription(VehicleInput, "~/input/vehicle_inputs", self.inputs_callback, 1)
        #publishers
        self.pub_objects = self.create_publisher(VehicleState, '/vehicle_state', 1)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)


    #CALLBACKS:
    def inputs_callback(self, msg):
        self.inputs = msg
        self.steering = self.inputs.steering
        self.throttle = self.inputs.throttle

    def accel_callback(self, msg):
        self.accel = msg

    def gyro_callback(self, msg):
        self.gyro = msg
        


    def ground_truth_callback(self, msg):
        self.gtx = msg.pose.position.x
        self.gty = msg.pose.position.y
        self.gtvx = msg.twist.linear.x
        self.gtvy = msg.twist.linear.y
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        z = msg.pose.orientation.z
        w = msg.pose.orientation.w
        
        

    def mag_callback(self,msg):
        self.mag = msg
        mag_x = self.mag.magnetic_field.x
        mag_y = self.mag.magnetic_field.y
        mag_z = self.mag.magnetic_field.z
        xGauss = mag_x*0.48828125
        yGauss = mag_y*0.4882815
        if(xGauss==0):
            if(yGauss<0):
                self.D = 0
            else:
                self.D = 90
        else:
            self.D = math.atan2(yGauss,xGauss)*180/math.pi
        while(self.D>360):
            self.D = self.D-360
        while(self.D<0):
            self.D = self.D+360
        
        if(not self.orig_heading_set):
            self.orig_heading_set = True
            self.graph.set_rotation(np.deg2rad(self.D) -self.init_theta)
            self.state[2,0] = self.init_theta


    def gps_callback(self,msg):
        self.gps = msg
        self.gps_ready = True
        if(math.isnan(self.gps.latitude)):
            #arbitrary values for when we don't get any data (in reality)
            self.lat = -10
            self.lon = -10
            self.alt = -10
        else:
            self.lat = self.gps.latitude
            self.lon = self.gps.longitude
            self.alt = self.gps.altitude

        if(not self.origin_set):
           self.origin_set = True
           self.graph.set_graph(self.lat,self.lon, self.alt)
        
        x,y,z = self.graph.gps2cartesian(self.lat,self.lon,self.alt)
        if(self.orig_heading_set):
            self.x,self.y, self.z =self.graph.rotate(x,y,z)
            self.x +=self.init_x
            self.y +=self.init_y



    #callback to run a loop and publish data this class generates
    def pub_callback(self):
        u = np.array([[self.throttle], [self.steering/2.2]])

        z = np.array([[self.x],[self.y], [np.deg2rad(self.D)]])
                
        if(self.SE_mode == "EKF"):
            self.EKFstep(u, z)
        elif(self.SE_mode == "PF"):
            self.PFstep(u,z)
        
        # with open('data.csv', 'a', encoding = 'UTF8') as csvfile:
        #    mywriter = csv.writer(csvfile)
        #    mywriter.writerow([self.gtx, self.gty, np.deg2rad(self.D), self.state[3,0], self.throttle, self.steering])
        #    csvfile.close()

       
        msg = VehicleState()
        #pos and velocity are in meters, from the origin, [x, y, z]
        if(self.SE_mode == "GT"):
            msg.pose.position.x = float(self.gtx)
            msg.pose.position.y = float(self.gty)
            msg.pose.orientation.z = np.deg2rad(self.D)
            msg.twist.linear.x = float(self.gtvx)
            msg.twist.linear.y = float(self.gtvy)
        else:
            msg.pose.position.x = float(self.state[0,0])
            msg.pose.position.y = float(self.state[1,0])
            msg.pose.orientation.z = float(self.state[2,0])
            msg.twist.linear.x = float(self.state[3,0]*math.cos(self.state[2,0]))
            msg.twist.linear.y = float(self.state[3,0]*math.sin(self.state[2,0]))
            


        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)
    
    def EKFstep(self, u, z):
        self.state = self.ekf.predict(self.state, u)
        if(self.gps_ready):
            self.state = self.ekf.correct(self.state, z)
            self.gps_ready = False

    def PFstep(self, u, z):
        self.state = self.pf.update(u, z)

def main(args=None):
    print("=== Starting State Estimation Node ===")
    rclpy.init(args=args)
    estimator = StateEstimationNode()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()