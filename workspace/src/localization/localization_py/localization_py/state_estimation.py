import csv
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
#from nav_msgs.msg import 
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
#from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt
import matplotlib
import math
import numpy as np


import sys
import os




#TODO: what does this do?
ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
sys.path.insert(0, os.path.abspath(ament_tools_root))

from EKF import EKF
from Chrono_coordinate_transfer import graph

class StateEstimationNode(Node):
    def __init__(self):
        super().__init__('state_estimation_node')
        
        self.declare_parameter('vis', False)
        self.vis = self.get_parameter('vis').get_parameter_value().bool_value

        #update frequency of this node
        self.freq = 10.0

        if(self.vis):
            matplotlib.use("TKAgg")
            #data that will be used by this class
            #TODO: a lot of these variables aren't used, this data needs to be cleaned up
            self.fig = plt.figure()
            self.fig.suptitle('Kalman Filter', fontsize = 20)
        self.gps = ""
        self.groundTruth = ""
        #self.imu = ""
        self.mag = ""
        #self.state = ""


        #For reading the inputs to the vehicle...
        self.declare_parameter("use_sim_msg", False)
        self.use_sim_msg = self.get_parameter("use_sim_msg").get_parameter_value().bool_value

        if self.use_sim_msg:
            global VehicleInput
            from chrono_ros_msgs.msg import ChDriverInputs as VehicleInput
        else:
            global VehicleInput
            from art_msgs.msg import VehicleInput



        #x, y, from measurements
        self.x = 0
        self.y = 0

        self.first_write = True

        self.prev_gt_t = []
        
        #what we will be using for our state vector. (x, y, theta yaw, v vel)
        self.state = np.zeros((4,1))
        np.vstack(self.state)
        #TODO: edit these to change the starting location...
        self.init_x = 5
        self.init_y = 5
        self.init_theta = 2.0
        self.state[0,0] = self.init_x
        self.state[1,0] = self.init_y
        self.state[2,0] = self.init_theta



        self.gps_ready = False

        #true X, True Y, velocity, History of each
        self.gty = 0
        self.gtx = 0
        self.gtz = 0

        #origin, and whether or not the origin has been set yet.
        self.origin = [0,0]
        self.origin_set = False

        self.orig_heading = 0
        self.orig_heading_set = False
        self.D = 0

        #IMU data, x, y, z acceleration
        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0


        #inputs to the vehicle
        self.throttle = 0
        self.steering = 0

        #time between imu updates, sec
        self.dt_gps = 0.1

        self.accumulated_error = 0

        self.ekf = EKF(self.dt_gps)
        
        sim = True

        #our graph object
        self.graph =  graph()

        #subscribers
        self.sub_gps = self.create_subscription(NavSatFix, '~/input/gps', self.gps_callback, 1)
        if(sim):
            self.sub_groundTruth = self.create_subscription(NavSatFix, '~/input/groundTruth', self.groundTruth_callback, 100000)
        self.sub_mag = self.create_subscription(MagneticField, "~/input/mag", self.mag_callback, 10)
        self.sub_gyro = self.create_subscription(Imu, "~/input/gyro", self.gyro_callback, 10)
        self.sub_accel = self.create_subscription(Imu, "~/input/accel", self.accel_callback, 100)
        self.sub_control = self.create_subscription(VehicleInput, "~/input/vehicleInput", self.inputs_callback, 10)
        #TODO: fix the line above
        #publishers
        #TODO: change callback time.
        self.pub_objects = self.create_publisher(VehicleState, '/vehicle_state', 100)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
        #TODO: how do we get times?
    #CALLBACKS:
    def inputs_callback(self, msg):
        self.inputs = msg
        self.steering = self.inputs.steering
        self.throttle = self.inputs.throttle
        #self.get_logger().info("throttle: "+ str(self.throttle)+ " steering: "+str(self.steering))

    def accel_callback(self, msg):
        self.accel = msg
        
    def gyro_callback(self, msg):
        self.gyro = msg
    def groundTruth_callback(self,msg):
        self.groundTruth = msg
        lat = self.groundTruth.latitude
        lon = self.groundTruth.longitude
        alt = self.groundTruth.altitude
        if(not self.origin_set):
           self.origin_set = True
           self.graph.set_graph(lat,lon, alt)
        x,y,z = self.graph.gps2cartesian(lat,lon,alt)
        if(self.orig_heading_set):
            self.gtx,self.gty, self.gtz =self.graph.rotate(x,y,z)
            self.gtx = self.gtx+self.init_x
            self.gty = self.gty+self.init_y
        
        
        

    def mag_callback(self,msg):
        self.mag = msg
        mag_x = self.mag.magnetic_field.x
        mag_y = self.mag.magnetic_field.y
        mag_z = self.mag.magnetic_field.z
        #TODO: is the 0.4 needed??
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
            self.orig_heading = self.D
            self.graph.set_rotation(np.deg2rad(self.D-14)-self.init_theta)


    def gps_callback(self,msg):
        self.gps = msg
        self.gps_ready = True
        # if(math.isnan(self.gps.latitude)):
        #     self.lat = -10
        #     self.lon = -10
        #     self.alt = -10
        # else:
        self.lat = self.gps.latitude
        self.lon = self.gps.longitude
        self.alt = self.gps.altitude

        # if(not self.origin_set):
        #    self.origin_set = True
        #    self.graph.set_graph(self.lat,self.lon, self.alt)
        # if(self.orig_heading_set and self.origin_set):
        #     self.x,self.y,self.z = self.graph.gps2cartesian(self.lat,self.lon,self.alt)
        #     self.x,self.y, self.z =self.graph.rotate(self.x,self.y,self.z)

        if(self.origin_set):
            x,y,z = self.graph.gps2cartesian(self.lat,self.lon,self.alt)
            if(self.orig_heading_set):
                self.x,self.y, self.z =self.graph.rotate(x,y,z)
                self.x +=self.init_x
                self.y +=self.init_y
        
        else:
           self.x = self.init_x
           self.y = self.init_y
           self.z = 0



    #callback to run a loop and publish data this class generates
    def pub_callback(self):
        u = np.array([[self.throttle], [self.steering/4]])
        z = np.array([[self.x],[self.y], [np.deg2rad(self.D)]])
        self.EKFstep(u, z)
        

        #self.get_logger().info("IS THE ORIGIN SET???" + str(self.origin_set))
        if(self.first_write):
           os.remove("data.csv")
           self.first_write = False

        self.get_logger().info("THE HEADING IS: " + str(self.state[2][0]))
        with open('data.csv', 'a', encoding = 'UTF8') as csvfile:
           mywriter = csv.writer(csvfile)
           mywriter.writerow([self.x, self.y, self.gtx, self.gty, self.state[0][0], self.state[1][0], self.D, self.throttle, self.steering])
           csvfile.close()

       
        msg = VehicleState()
        #pos and velocity are in meters, from the origin, [x, y, z]
        #TODO: is this right?
        if(self.state.ndim == 2):
            #msg.pose.position.x = float(self.x)
            #msg.pose.position.y = float(self.y)
            msg.pose.position.x = float(self.state[0][0])
            msg.pose.position.y = float(self.state[1][0])
            msg.pose.orientation.z = float(self.state[2][0])
            #TODO: make sure these are correct
            msg.twist.linear.x = float(self.state[3][0]*math.cos(self.state[2][0]))
            msg.twist.linear.y = float(self.state[3][0]*math.sin(self.state[2][0]))

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)
    
    def EKFstep(self, u, z):
        self.state = self.ekf.predict(self.state, u)
        if(self.gps_ready):
            self.state = self.ekf.correct(self.state, z)
            self.gps_ready = False

def main(args=None):
    print("=== Starting State Estimation Node ===")
    rclpy.init(args=args)
    estimator = StateEstimationNode()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()













# import csv
# import rclpy
# from rclpy.node import Node
# from art_msgs.msg import VehicleState
# #from nav_msgs.msg import 
# from sensor_msgs.msg import Imu, NavSatFix, MagneticField
# #from sensor_msgs.msg import NavSatFix
# from ament_index_python.packages import get_package_share_directory

# import matplotlib.pyplot as plt
# import matplotlib
# import math
# import numpy as np


# import sys
# import os




# #TODO: what does this do?
# ament_tools_root = os.path.join(os.path.dirname(__file__), '.')
# sys.path.insert(0, os.path.abspath(ament_tools_root))

# from EKF import EKF
# from Chrono_coordinate_transfer import graph

# class StateEstimationNode(Node):
#     def __init__(self):
#         super().__init__('state_estimation_node')
        
#         self.declare_parameter('vis', False)
#         self.vis = self.get_parameter('vis').get_parameter_value().bool_value

#         #update frequency of this node
#         self.freq = 10.0

#         if(self.vis):
#             matplotlib.use("TKAgg")
#             #data that will be used by this class
#             #TODO: a lot of these variables aren't used, this data needs to be cleaned up
#             self.fig = plt.figure()
#             self.fig.suptitle('Kalman Filter', fontsize = 20)
#         self.gps = ""
#         self.groundTruth = ""
#         #self.imu = ""
#         self.mag = ""
#         #self.state = ""


#         #For reading the inputs to the vehicle...
#         self.declare_parameter("use_sim_msg", False)
#         self.use_sim_msg = self.get_parameter("use_sim_msg").get_parameter_value().bool_value

#         if self.use_sim_msg:
#             global VehicleInput
#             from chrono_ros_msgs.msg import ChDriverInputs as VehicleInput
#         else:
#             global VehicleInput
#             from art_msgs.msg import VehicleInput



#         #x, y, from measurements
#         self.x = 0
#         self.y = 0
#         self.hx = [0]
#         self.hy = [0]

#         self.first_write = True

#         self.prev_gt_t = []
        
#         #what we will be using for our state vector. (x, y, theta yaw, v vel)
#         self.state = np.zeros((4,1))
#         np.vstack(self.state)

#         self.gps_ready = False

#         #true X, True Y, velocity, History of each
#         self.gty = 0
#         self.gtx = 0
#         self.gtz = 0

#         #origin, and whether or not the origin has been set yet.
#         self.origin = [0,0]
#         self.origin_set = False

#         self.orig_heading = 0
#         self.orig_heading_set = False
#         self.D = 0

#         #IMU data, x, y, z acceleration
#         self.x_accel = 0
#         self.y_accel = 0
#         self.z_accel = 0


#         #inputs to the vehicle
#         self.throttle = 0
#         self.steering = 0

#         #time between imu updates, sec
#         self.dt_gps = 0.1

#         self.accumulated_error = 0

#         self.ekf = EKF(self.dt_gps)
        
#         sim = True

#         #our graph object
#         self.graph =  graph()

#         #subscribers
#         self.sub_gps = self.create_subscription(NavSatFix, '~/input/gps', self.gps_callback, 10)
#         if(sim):
#             self.sub_groundTruth = self.create_subscription(NavSatFix, '~/input/groundTruth', self.groundTruth_callback, 100000)
#         self.sub_mag = self.create_subscription(MagneticField, "~/input/mag", self.mag_callback, 10)
#         self.sub_gyro = self.create_subscription(Imu, "~/input/gyro", self.gyro_callback, 10)
#         self.sub_accel = self.create_subscription(Imu, "~/input/accel", self.accel_callback, 100)
#         self.sub_control = self.create_subscription(VehicleInput, "~/input/vehicleInput", self.inputs_callback, 10)
#         #TODO: fix the line above
#         #publishers
#         #TODO: change callback time.
#         self.pub_objects = self.create_publisher(VehicleState, '/vehicle_state', 100)
#         self.timer = self.create_timer(1/self.freq, self.pub_callback)
#         #TODO: how do we get times?
#     #CALLBACKS:
#     def inputs_callback(self, msg):
#         self.inputs = msg
#         self.steering = self.inputs.steering
#         self.throttle = self.inputs.throttle
#         #self.get_logger().info("throttle: "+ str(self.throttle)+ " steering: "+str(self.steering))

#     def accel_callback(self, msg):
#         self.accel = msg
        
#     def gyro_callback(self, msg):
#         self.gyro = msg
#     def groundTruth_callback(self,msg):
#        self.groundTruth = msg
#        lat = self.groundTruth.latitude
#        lon = self.groundTruth.longitude
#        alt = self.groundTruth.altitude
#        #DONT PUSH WITH THIS BECAUSE MOST DEMOS DONT HAVE THE GT
#        if(not self.origin_set):
#            self.origin_set = True
#            self.graph.set_graph(lat,lon, alt)
#        x,y,z = self.graph.gps2cartesian(lat,lon,alt)
#        if(self.orig_heading_set):
#             self.gtx,self.gty, self.gtz =self.graph.rotate(x,y,z)
        
        
        

#     def mag_callback(self,msg):
#         self.mag = msg
#         mag_x = self.mag.magnetic_field.x
#         mag_y = self.mag.magnetic_field.y
#         mag_z = self.mag.magnetic_field.z
#         #TODO: is the 0.4 needed??
#         xGauss = mag_x*0.48828125
#         yGauss = mag_y*0.4882815
#         if(xGauss==0):
#             if(yGauss<0):
#                 self.D = 0
#             else:
#                 self.D = 90
#         else:
#             self.D = math.atan2(yGauss,xGauss)*180/math.pi
#         while(self.D>360):
#             self.D = self.D-360
#         while(self.D<0):
#             self.D = self.D+360
#         if(not self.orig_heading_set):
#             self.orig_heading_set = True
#             self.orig_heading = self.D
#             self.graph.set_rotation(np.deg2rad(self.D-14))


#     def gps_callback(self,msg):
#        self.gps = msg
#        self.gps_ready = True
#        self.lat = self.gps.latitude
#        self.lon = self.gps.longitude
#        self.alt = self.gps.altitude
#        if(self.origin_set):
#             x,y,z = self.graph.gps2cartesian(self.lat,self.lon,self.alt)
#             if(self.orig_heading_set):
#                 self.x,self.y, self.z =self.graph.rotate(x,y,z)
#        else:
#            self.x = 0
#            self.y = 0
#            self.z = 0

#     #callback to run a loop and publish data this class generates
#     def pub_callback(self):
#         u = np.array([[self.throttle], [self.steering/4]])
#         z = np.array([[self.x],[self.y], [np.deg2rad(self.D)]])
#         self.EKFstep(u, z)
        


#         if(self.first_write):
#            os.remove("data.csv")
#            self.first_write = False


#         with open('data.csv', 'a', encoding = 'UTF8') as csvfile:
#            mywriter = csv.writer(csvfile)
#            mywriter.writerow([self.x, self.y, self.gtx, self.gty, self.state[0][0], self.state[1][0], self.D, self.throttle, self.steering])
#            csvfile.close()

       
#         msg = VehicleState()
#         #pos and velocity are in meters, from the origin, [x, y, z]
#         #TODO: is this right?
#         if(self.state.ndim == 2):
#             #msg.pose.position.x = float(self.x)
#             #msg.pose.position.y = float(self.y)
#             msg.pose.position.x = float(self.state[0][0])
#             msg.pose.position.y = float(self.state[1][0])
#             msg.pose.orientation.z = float(self.state[2][0])
#             #TODO: make sure these are correct
#             msg.twist.linear.x = float(self.state[3][0]*math.cos(self.state[2][0]))
#             msg.twist.linear.y = float(self.state[3][0]*math.sin(self.state[2][0]))

#         msg.header.stamp = self.get_clock().now().to_msg()
#         self.pub_objects.publish(msg)
    
#     def EKFstep(self, u, z):
#         self.state = self.ekf.predict(self.state, u)
#         if(self.gps_ready):
#             self.state = self.ekf.correct(self.state, z)
#             self.gps_ready = False

# def main(args=None):
#     print("=== Starting State Estimation Node ===")
#     rclpy.init(args=args)
#     estimator = StateEstimationNode()
#     rclpy.spin(estimator)
#     estimator.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()