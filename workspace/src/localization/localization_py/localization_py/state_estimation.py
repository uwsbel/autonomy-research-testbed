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

from KalmanFilter import KalmanFilter
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
        self.state = ""

        #mag measurements
        self.mag_x = 0
        self.mag_y = 0
        self.mag_z = 0
        #lat, lon, ground truth of each
        self.lat = 0
        self.lon = 0
        self.gt_lat = 0
        self.gt_lon = 0

        #x, y, kf predictions of each, history of each
        self.x = 0
        self.y = 0
        self.kfx = 0
        self.kfy = 0
        self.hkfx = []
        self.hkfy = []
        self.hx = []
        self.hy = []

        #true X, True Y, History of each
        self.ty = 0
        self.tx = 0
        self.htx = []
        self.hty = []

        #origin, and whether or not the origin has been set yet.
        self.origin = [0,0]
        self.origin_set = False

        #IMU data, x, y, z acceleration
        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0


        #time between imu updates, sec
        self.dt_imu = .1

        self.accumulated_error = 0

        
        sim = True

        #our graph object
        self.graph =  graph()
        #TODO: Still need to do some parameter fitting for this
        #Kalman Filter for x and y directions, and multiplier for their data.
        self.kf_x = KalmanFilter(0.2, 0.1, 1.2, 1)
        self.kf_y = KalmanFilter(0.2, 0.1, 1.2, 1)
        self.kf_data_multiplier = 10

        #subscribers
        self.sub_gps = self.create_subscription(NavSatFix, '~/input/gps', self.gps_callback, 1)
        if(sim):
            self.sub_groundTruth = self.create_subscription(NavSatFix, '~/input/groundTruth', self.groundTruth_callback, 1)
        self.sub_mag = self.create_subscription(MagneticField, "~/input/magnetometer", self.mag_callback, 10)
        self.sub_gyro = self.create_subscription(Imu, "~/input/gyroscope", self.gyro_callback, 10)
        self.sub_accel = self.create_subscription(Imu, "~/input/accelerometer", self.accel_callback, 10)

        #publishers
        #TODO: change callback time.
        self.pub_objects = self.create_publisher(VehicleState, '~/output/vehicle_state', 1)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
    #CALLBACKS:
    def accel_callback(self, msg):
        self.accel = msg
    def gyro_callback(self, msg):
        self.gyro = msg
    def groundTruth_callback(self,msg):
        self.groundTruth = msg

    def mag_callback(self,msg):
        self.mag = msg
        self.mag_x = self.mag.magnetic_field.x
        self.mag_y = self.mag.magnetic_field.y
        self.mag_z = self.mag.magnetic_field.z
        #TODO: is the 0.4 needed??
        xGauss = self.mag_x*0.48828125
        yGauss = self.mag_y*0.4882815
        if(xGauss==0):
            if(yGauss<0):
                D = 0
            else:
                D = 90
        else:
            D = math.atan2(yGauss,xGauss)*180/math.pi
        while(D>360):
            D = D-360
        while(D<0):
            D = D+360

    def gps_callback(self,msg):
        self.gps = msg
        self.lat = self.gps.latitude
        self.lon = self.gps.longitude
        self.alt = self.gps.altitude
        if(not self.origin_set):
            self.origin_set = True
            self.graph.set_graph(self.lat,self.lon, self.alt)
        self.x,self.y, self.z =self.graph.gps2cartesian(self.lat,self.lon,self.alt)
        self.hx.append(self.x)
        self.hy.append(self.y)



    
    #callback to run a loop and publish data this class generates
    def pub_callback(self):
        self.KFstep()
        
        self.hkfx.append(self.kfx)
        self.hkfy.append(self.kfy)
        if(self.vis):
            plt.cla()
            plt.plot(self.hkfx, self.hkfy, label = 'KF predictions', color = 'b', linewidth = 0.5)
        #   plt.plot(self.htx, self.hty, label = 'True Position', color = 'g', linewidth = 0.5)
            plt.plot(self.hx, self.hy, label='Measured Position', color = 'r')
            plt.xlabel('Position_x (m)', fontsize=20)
            plt.ylabel('Position_y (m)', fontsize=20)
            plt.legend()
            plt.draw()
            plt.pause(0.0001)
        error = math.sqrt((self.x)**2 + self.y**2) - math.sqrt(self.kfx**2+self.kfy**2)
        self.accumulated_error = self.accumulated_error + abs(error)


        msg = VehicleState()
        #pos and velocity are in meters, from the origin, [x, y, z]
        #TODO: is this right?
        msg.pose.position.x = float(self.kfx)
        msg.pose.position.y = float(self.kfy)

        # add header timestamps
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)
    def KFstep(self):
        self.kf_x.update(self.x*self.kf_data_multiplier)
        self.kf_y.update(self.y*self.kf_data_multiplier)
        self.kfx = self.kf_x.predict()[0]/self.kf_data_multiplier
        self.kfy = self.kf_y.predict()[0]/self.kf_data_multiplier
def main(args=None):
    print("=== Starting State Estimation Node ===")
    rclpy.init(args=args)
    estimator = StateEstimationNode()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
