import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
#from nav_msgs.msg import 
#from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from sensor_msgs.msg import NavSatFix
from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt
import matplotlib
import math


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

        #update frequency of this node
        self.freq = 10.0
        matplotlib.use("TKAgg")
        #data that will be used by this class
        #TODO: a lot of these variables aren't used, this data needs to be cleaned up
        self.fig = plt.figure()
        self.fig.suptitle('Kalman Filter', fontsize = 20)
        self.gps = ""
        self.groundTruth = ""
        self.imu = ""
        self.mag = ""
        self.state = ""

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

        #x, y, z "dead reckoning" velocity for imu
        self.x_dr_v = 0
        self.y_dr_v = 0
        self.z_dr_v = 0

        # velocity, yaw "dead reconing" for imu
        self.v_dr = 0
        self.yaw_dr = 0

        #time between imu updates, sec
        self.dt_imu = .1

        self.accumulated_error = 0

        


        #our graph object
        self.graph =  graph()
        #TODO: Still need to do some parameter fitting for this
        #Kalman Filter for x and y directions, and multiplier for their data.
        self.kf_x = KalmanFilter(0.2, 0.1, 1.2, 1)
        self.kf_y = KalmanFilter(0.2, 0.1, 1.2, 1)
        self.kf_data_multiplier = 10

        #subscribers
        self.sub_gps = self.create_subscription(NavSatFix, '~/input/gps', self.gps_callback, 1)
        #self.sub_imu = self.create_subscription(Imu, 'miniav/imu', self.imu_callback, 10)
        #self.sub_mag = self.create_subscription(MagneticField, "magnetic", self.mag_callback, 10)
        #self.sub_groundTruth = self.create_subscription(NavSatFix, 'miniav/groundTruth', self.groundTruth_callback, 10)

        #publishers
        #TODO: is this the right place to output or should it be /output/vehicle_state? and change callback time.
        self.pub_objects = self.create_publisher(VehicleState, '~/output/vehicle_state', 1)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)
    #TANGENT PLANE MATH:


    
    #x = r(cos(lat))(cos(lon))
    #y = r(cos(lat)(sin(lon))
    #z = r(sin(lat))
    #Equation of Sphere: 0 = x^2+y^2+z^2-12,742,000 (in meters)
    #F(x,y,z) = x^2+y^2+z^2-12,742,000 = 0
    #G() = Gradient
    #G()
    #THIS IS THE NEW GPS CALLBACK, USING THE COORDINATE TRANSFER
    def gps_callback(self,msg):
        self.gps = msg
        self.lat = self.gps.latitude
        self.lon = self.gps.longitude
        self.alt = self.gps.altitude
        if(not self.origin_set):
            self.origin_set = True
            self.graph.set_graph(self.lat,self.lon, self.alt)
        self.x,self.y, self.z =self.lon, self.lat, self.alt # self.graph.gps2cartesian(self.lat,self.lon,self.alt)
        #self.get_logger().info("["+str(self.lat)+","+str(self.lon)+"]")
        self.get_logger().info("["+str(self.x)+","+str(self.y)+"]")
        self.hx.append(self.x)
        self.hy.append(self.y)



    #function to process data this class subscribes to
    #def gps_callback(self, msg):
    #    self.gps = msg
    #    self.lat = self.gps.latitude
    #    self.lon = self.gps.longitude
    #    if(not self.origin_set):
    #        self.origin_set = True
    #        self.origin = [self.lat, self.lon]
    #    self.x = (self.lat-self.origin[0])*111139
    #    self.y = (self.lon-self.origin[1])*111139
    #    self.get_logger().info("["+str(self.lat)+","+str(self.lon)+"]")
    #    self.hx.append(self.x)
    #    self.hy.append(self.y)
        
            
            #hx_hold = self.hx
            #self.hx = [self.x, hx_hold[0],
            #         hx_hold[1], hx_hold[2], hx_hold[3]]
            #hy_hold = self.hy
            #self.hy = [self.y, hy_hold[0],
            #        hy_hold[1], hy_hold[2], hy_hold[3]]
        #self.get_logger().info("X: "+str(self.x)+ " Y: "+str(self.y))

    #def groundTruth_callback(self, msg):
    #    self.groundTruth = msg
    #    self.gt_lat = self.groundTruth.latitude
    #    self.gt_lon = self.groundTruth.longitude
    ##    if(not self.origin_set):
     #       self.origin_set = True
     #       self.origin = [self.gt_lat, self.gt_lon]
     #   self.tx = (self.gt_lat-self.origin[0])*111139
     #   self.ty = (self.gt_lon-self.origin[1])*111139

#        self.htx.append(self.tx)
 #       self.hty.append(self.ty)
        #htx_hold = self.htx
        #self.htx = [self.tx, htx_hold[0],
        #             htx_hold[1], htx_hold[2], htx_hold[3]]
        #hty_hold = self.hty
        #self.hty = [self.ty, hty_hold[0],
        #             hty_hold[1], hty_hold[2], hty_hold[3]]

        #self.get_logger().info("noise X: "+str(self.lat_noisem)+" noise Y: "+str(self.lon_noisem))
  #  def imu_callback(self, msg):
   #     self.imu = msg
    #    self.x_accel = self.imu.linear_acceleration.x
    #    self.y_accel = self.imu.linear_acceleration.y
    #    self.z_accel = self.imu.linear_acceleration.z

#        self.x_dr_v = self.x_dr_v +self.x_accel*self.dt_imu
 #       self.y_dr_v = self.y_dr_v +self.y_accel*self.dt_imu
  #      self.z_dr_v = self.z_dr_v +self.z_accel*self.dt_imu

        #For now we assume we are driving on a 2D plane, only use x and y
        #TODO: may need to change to 3D later
   #     self.v_dr = math.sqrt(self.x_dr_v**2 + self.y_dr_v**2)

    #    self.get_logger().info("x acceleration"+str(self.x_accel))  

    #def mag_callback(self, msg):
    #    self.mag = msg

    #callback to run a loop and publish data this class generates
    def pub_callback(self):
        self.KFstep()
        
        self.hkfx.append(self.kfx)
        self.hkfy.append(self.kfy)
        #hkfx_hold = self.hkfx
        #self.hkfx = [self.kfx, hkfx_hold[0], hkfx_hold[1], hkfx_hold[2], hkfx_hold[3]]
        #hkfy_hold = self.hkfy
        #self.hkfy = [self.kfy, hkfy_hold[0], hkfy_hold[1], hkfy_hold[2], hkfy_hold[3]]

        plt.cla()
        #plt.plot(self.hkfx, self.hkfy, label = 'KF predictions', color = 'b', linewidth = 0.5)
     #   plt.plot(self.htx, self.hty, label = 'True Position', color = 'g', linewidth = 0.5)
        plt.scatter(self.hx, self.hy, label='Measured Position', color = 'r')#, linewidth = 0.5)
        plt.xlabel('Position_x (m)', fontsize=20)
        plt.ylabel('Position_y (m)', fontsize=20)
        plt.legend()
        plt.draw()
        plt.pause(0.0001)
        #self.get_logger().info("["+str(self.kfx)+","+str(self.kfy)+"]")
        #error = (self.x)**2
        error = math.sqrt((self.x)**2 + self.y**2) - math.sqrt(self.kfx**2+self.kfy**2)
        self.accumulated_error = self.accumulated_error + abs(error)
        #self.get_logger().info(str(self.accumulated_error))


        msg = VehicleState()
        #pos and velocity are in meters, from the origin, [x, y, z]
        msg.pose.position.x = float(self.kfx)
        msg.pose.position.y = float(self.kfy)
        # [float(self.kfx), float(self.kfy), -1.]
        #msg.velocity = [-1., -1., -1.]
        self.pub_objects.publish(msg)
        #self.get_logger().info('Publishing')
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
