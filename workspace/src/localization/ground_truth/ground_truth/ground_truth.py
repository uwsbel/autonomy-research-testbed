import csv
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from chrono_ros_interfaces.msg import Body as ChVehicle
import matplotlib.pyplot as plt
import matplotlib
import math
import numpy as np
import sys
import os
from enum import Enum
from localization_shared_utils import get_dynamics, get_coordinate_transfer


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__("state_estimation_node")

        # ROS PARAMETERS
        self.use_sim_msg = (
            self.get_parameter("use_sim_time").get_parameter_value().bool_value
        )

        # update frequency of this node
        self.freq = 10.0

        self.gps = ""
        self.ground_truth = ""
        self.mag = ""

        # x, y, from measurements
        self.x = 0
        self.y = 0

        # what we will be using for our state vector. (x, y, theta yaw, v vel)
        self.state = np.zeros((4, 1))

        self.init_x = 0.0
        self.init_y = 0.0
        self.init_theta = 0.0
        self.init_v = 0.0
        self.state[0, 0] = self.init_x
        self.state[1, 0] = self.init_y
        self.state[2, 0] = self.init_theta
        self.state[3, 0] = self.init_v

        self.gps_ready = False

        # ground truth velocity
        self.gtvy = 0
        self.gtvx = 0
        self.D = 0

        # origin, and whether or not the origin has been set yet.
        self.origin_set = False
        self.orig_heading_set = False

        # inputs to the vehicle
        self.throttle = 0.0
        self.steering = 0

        # time between imu updates, sec
        self.dt_gps = 1 / self.freq

        # our graph object, for reference frame
        self.graph = get_coordinate_transfer()

        # subscribers
        self.sub_gps = self.create_subscription(
            NavSatFix, "~/input/gps", self.gps_callback, 1
        )

        self.sub_mag = self.create_subscription(
            MagneticField, "~/input/magnetometer", self.mag_callback, 1
        )
        self.sub_control = self.create_subscription(
            VehicleInput, "~/input/vehicle_inputs", self.inputs_callback, 1
        )
        # publishers
        self.pub_objects = self.create_publisher(
            VehicleState, "~/output/filtered_state", 1
        )
        self.timer = self.create_timer(1 / self.freq, self.pub_callback)

    # CALLBACKS:
    def inputs_callback(self, msg):
        self.inputs = msg
        self.steering = self.inputs.steering
        self.throttle = self.inputs.throttle

    def mag_callback(self, msg):
        self.mag = msg
        mag_x = self.mag.magnetic_field.x
        mag_y = self.mag.magnetic_field.y
        mag_z = self.mag.magnetic_field.z
        xGauss = mag_x * 0.48828125
        yGauss = mag_y * 0.4882815
        if xGauss == 0:
            if yGauss < 0:
                self.D = 0
            else:
                self.D = 90
        else:
            self.D = math.atan2(yGauss, xGauss) * 180 / math.pi
        while self.D > 360:
            self.D = self.D - 360
        while self.D < 0:
            self.D = self.D + 360

        if not self.orig_heading_set:
            self.orig_heading_set = True
            self.graph.set_rotation(np.deg2rad(self.D) - self.init_theta)
            self.state[2, 0] = self.init_theta

    def gps_callback(self, msg):
        self.gps = msg
        self.gps_ready = True
        if math.isnan(self.gps.latitude):
            # arbitrary values for when we don't get any data (in reality)
            self.lat = -10
            self.lon = -10
            self.alt = -10
        else:
            self.lat = self.gps.latitude
            self.lon = self.gps.longitude
            self.alt = self.gps.altitude

        if not self.origin_set:
            self.origin_set = True
            self.graph.set_graph(self.lat, self.lon, self.alt)

        x, y, z = self.graph.gps2cartesian(self.lat, self.lon, self.alt)
        if self.orig_heading_set:
            newx, newy, newz = self.graph.rotate(x, y, z)
            self.gtvx, self.gtvy = (newx - self.x) / self.dt_gps, (
                newy - self.y
            ) / self.dt_gps
            self.x, self.y, self.z = newx, newy, newz
            self.x += self.init_x
            self.y += self.init_y

    # callback to run a loop and publish data this class generates
    def pub_callback(self):
        msg = VehicleState()
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)
        # TODO: this should be a quat in the future, not the heading.
        msg.pose.orientation.z = np.deg2rad(self.D)
        msg.twist.linear.x = float(self.gtvx)
        msg.twist.linear.y = float(self.gtvy)

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)


def main(args=None):
    print("=== Starting State Estimation Node ===")
    rclpy.init(args=args)
    estimator = GroundTruthNode()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()