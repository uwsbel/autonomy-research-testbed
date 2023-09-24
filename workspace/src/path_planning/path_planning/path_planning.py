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
from art_perception_msgs.msg import ObjectArray, Object
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.interpolate import interp1d,splev,splprep
import os
import json

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        #update frequency of this node
        self.freq = 10.0

        # READ IN SHARE DIRECTORY LOCATION
        package_share_directory = get_package_share_directory('path_planning')

        # READ IN PARAMETERS
        self.declare_parameter('planning_mode', "cones")
        self.planning_mode = self.get_parameter('planning_mode').get_parameter_value().string_value

        self.declare_parameter('vis', False)
        self.vis = self.get_parameter('vis').get_parameter_value().bool_value

        self.declare_parameter('lookahead', 2.0)
        self.lookahead = self.get_parameter('lookahead').get_parameter_value().double_value
        
        if(self.planning_mode == "waypoints"):
            self.declare_parameter('path_file', "/home/art/art/workspace/src/path_planning/path_planning/path.csv")
            path_file = self.get_parameter('path_file').get_parameter_value().string_value
            file = open(path_file)
            self.ref_traj = np.loadtxt(file, delimiter=",")

        #data that will be used by this class
        self.state = VehicleState()
        self.path = Path()
        # self.objects = ObjectArray()

        self.green_cones = np.array([])
        self.red_cones = np.array([])

        self.go = False

        #subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        # self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        if(self.planning_mode == "waypoints"):
            self.sub_state = self.create_subscription(VehicleState, '~/input/vehicle_state', self.state_callback, qos_profile)
        else:
            self.sub_objects = self.create_subscription(ObjectArray, '~/input/objects', self.objects_callback, qos_profile)

            

        if self.vis:
            matplotlib.use("TKAgg")
            self.fig, self.ax = plt.subplots()
            plt.title("Path Planning")
            self.patches = []
            self.ax.set_xlim((-1,11))
            self.ax.set_ylim((-6,6))
            self.left_boundary = None
            self.right_boundary = None
            
        #publishers
        if(self.planning_mode == "waypoints"):
            self.pub_err_state = self.create_publisher(VehicleState, '~/output/error_state', 10)
        else:
            self.pub_path = self.create_publisher(Path, '~/output/path', 10)
        self.timer = self.create_timer(1/self.freq, self.pub_callback)

    #function to process data this class subscribes to
    def state_callback(self, msg):
        # self.get_logger().info("Received '%s'" % msg)
        self.go = True
        self.state = msg

    def objects_callback(self, msg):
        # self.get_logger().info("Received '%s'" % msg)
        # self.objects = msg

        self.go = True
    
        self.green_cones = []
        self.red_cones = []

        # self.get_logger().info("Detected cones: %s" % (str(len(msg.objects))))

        for obj in msg.objects:
            pos = [obj.pose.position.x,obj.pose.position.y,obj.pose.position.z]
            id = obj.classification.classification

            #calculate position from camera parameters, rect, and distance
            if(np.linalg.norm(pos) < 2.0*self.lookahead):
                if(id == 1):
                    self.red_cones.append(pos)
                elif(id == 2):
                    self.green_cones.append(pos)
                else:
                    self.get_logger().info("Object with unknown label detected {}".format(id))
        

    def order_cones(self,cones,start):
        ordered_cones = [start]

        ego = start
        for i in range(len(cones)):
            dist_2 = np.sum((cones - ego)**2, axis=1)
            id = np.argmin(dist_2)
            ordered_cones.append(cones[id,:])
            cones = np.delete(cones,id,axis=0)
        
        ordered_cones = np.asarray(ordered_cones)
        total_dist = 0
        for i in range(len(ordered_cones)-1):
            total_dist += np.linalg.norm(ordered_cones[i,:] -  ordered_cones[i+1,:])

        return ordered_cones, total_dist
        

    def plan_path(self):
        self.red_cones = np.asarray(self.red_cones)
        self.green_cones = np.asarray(self.green_cones)

        if(len(self.red_cones) == 0):
            self.red_cones = np.asarray([1,-1.5,0]) #phantom cone to right if none are seen
        if(len(self.green_cones) == 0):
            self.green_cones = np.asarray([1,1.5,0]) #phantom cone to right if none are seen

        self.red_cones = self.red_cones.reshape((-1,3))
        self.green_cones = self.green_cones.reshape((-1,3))

        left, l_dist = self.order_cones(self.green_cones,np.array([0.0,.5,0]))
        right, r_dist = self.order_cones(self.red_cones,np.array([0.0,-.5,0]))
        
        max_dist = 4

        left_spline,u = splprep(left[:,0:2].transpose(),k=max(1,min(int(len(left)/2),5)))
        left_samples = np.linspace(0, max_dist / l_dist, 100)
        b_left = splev(left_samples,left_spline)

        right_spline,u = splprep(right[:,0:2].transpose(),k=max(1,min(int(len(right)/2),5)))
        right_samples = np.linspace(0, max_dist / r_dist, 100)
        b_right = splev(right_samples,right_spline)

        center_line = np.array([(b_left[0] + b_right[0]) / 2, (b_left[1] + b_right[1]) / 2])
        # center_line = center_line[:,min(len(b_right[0]),len(b_left[0]))]
        
        distances = np.sum((center_line)**2, axis=0)
        id = np.argmin(np.abs(distances - self.lookahead**2))
        target_pt = center_line[:,id]

        # self.get_logger().info("B Left Spline: %s" % (str(len(b_left))))
        
        if(self.vis):
            [p.remove() for p in self.patches]
            self.patches.clear()

            if(self.left_boundary == None):
                self.left_boundary, = self.ax.plot(b_left[0],b_left[1],c='g')
            else:
                self.left_boundary.set_data(b_left[0],b_left[1])

            if(self.right_boundary == None):
                self.right_boundary, = self.ax.plot(b_right[0],b_right[1],c='r')
            else:
                self.right_boundary.set_data(b_right[0],b_right[1])

            for pos in right:
                circ = patches.Circle(pos[0:2],radius=.1,color='r')
                self.ax.add_patch(circ)
                self.patches.append(circ)

            for pos in left:
                circ = patches.Circle(pos[0:2],radius=.1,color='g')
                self.ax.add_patch(circ)
                self.patches.append(circ)

            circ = patches.Circle(target_pt,radius=.1,color='b')
            self.ax.add_patch(circ)
            self.patches.append(circ)

        return target_pt

    def wpts_path_plan(self):
        x_current = self.state.pose.position.x
        y_current = self.state.pose.position.y
        #theta_current = np.arctan2( 2*(x*w+z*y), x**2-w**2+y**2-z**2 )
        theta_current = self.state.pose.orientation.z-0.24845347641462115
        while theta_current<-np.pi:
            theta_current = theta_current+2*np.pi
        while theta_current>np.pi:
            theta_current = theta_current - 2*np.pi

        v_current = np.sqrt(self.state.twist.linear.x**2+self.state.twist.linear.y**2)

        #self.get_logger().info("SPEED: "+str(v_current))

        dist = np.zeros((1,len(self.ref_traj[:,1])))
        for i in range(len(self.ref_traj[:,1])):
            dist[0][i] = dist[0][i] = (x_current+np.cos(theta_current)*self.lookahead-self.ref_traj[i][0])**2+(y_current+np.sin(theta_current)*self.lookahead-self.ref_traj[i][1])**2
        index = dist.argmin()

        ref_state_current = list(self.ref_traj[index,:])
        err_theta = 0
        ref = ref_state_current[2]
        act = theta_current

        if( (ref>0 and act>0) or (ref<=0 and act <=0)):
            err_theta = ref-act
        elif( ref<=0 and act > 0):
            if(abs(ref-act)<abs(2*np.pi+ref-act)):
                err_theta = -abs(act-ref)
            else:
                err_theta = abs(2*np.pi + ref- act)
        else:
            if(abs(ref-act)<abs(2*np.pi-ref+act)):
                err_theta = abs(act-ref)
            else: 
                err_theta = -abs(2*np.pi-ref+act)


        RotM = np.array([ 
            [np.cos(-theta_current), -np.sin(-theta_current)],
            [np.sin(-theta_current), np.cos(-theta_current)]
        ])

        errM = np.array([[ref_state_current[0]-x_current],[ref_state_current[1]-y_current]])

        errRM = RotM@errM


        error_state = [errRM[0][0],errRM[1][0],err_theta, ref_state_current[3]-v_current]

        return error_state, ref_state_current[3]


    #callback to run a loop and publish data this class generates
    def pub_callback(self):
        if(not self.go):
            return
        if(self.planning_mode == "waypoints"):
            msg = VehicleState()
            error_state,ref_vel = self.wpts_path_plan()

            msg.pose.position.x = error_state[0]
            msg.pose.position.y = error_state[1]
            msg.pose.orientation.z = error_state[2]
            msg.twist.linear.x = error_state[3]
            msg.twist.linear.y = ref_vel
            self.pub_err_state.publish(msg)  

        else:
            msg = Path()
            
            target_pt = self.plan_path()

            #calculate path from current cone locations
            if(self.vis):
                plt.draw()
                plt.pause(0.0001)

            pt = PoseStamped()
            pt.pose.position.x = target_pt[0]
            pt.pose.position.y = target_pt[1]
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.poses.append(pt)
            self.pub_path.publish(msg)

def main(args=None):
    # print("=== Starting Path Planning Node ===")
    rclpy.init(args=args)
    planner = PathPlanningNode()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
