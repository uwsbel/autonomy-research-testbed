import csv
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState
from chrono_ros_interfaces.msg import DriverInputs as VehicleInput
from sensor_msgs.msg import Imu, NavSatFix, MagneticField
from chrono_ros_interfaces.msg import Body as ChVehicle
from nav_msgs.msg import Odometry  # Import the Odometry message
import matplotlib.pyplot as plt
import matplotlib
import math
import numpy as np
import sys
import os
from enum import Enum
from localization_shared_utils import get_dynamics, get_coordinate_transfer
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped


class GroundTruthNode(Node):
    """A state estimation node based that just pushes data from the observations.

    This state estimation package takes the GPS and Magnetometer observations and projects the location and heading of the vehicle onto a Local Tangent Plane (LTP) defined at the original location and orientation of the vehicle. The vehicle's speed is estimated by just taking the derivative of the positions.

    Attributes:
        init_x, init_y, init_theta: Initialization data for the definition of the local tangent plane on which the vehicle is assumed to drive.
        x, y: The Local Tangent Plane (LTP) - translated GPS coordinates.
        state: The 4 DOF state of the vehicle, as defined by its x and y coordinates, heading angle, and speed.
        gps: The observation of the position as a GPS reading.
        mag: The observation of the heading as a Magnetometer reading.origin_set: whether or not the origin and orientation of the LTP has been set.
        origin_heading_set: whether or not the original heading has been determined.
    """

    def __init__(self):
        """Initialize the state estimation node.

        Initialize the node and set the initial state, and initial sensor reading variables. Subscribe to the GPS and Magnetometer topics. Set up the publisher to publish to the `filtered_state` topic.
        """
        super().__init__("state_estimation_node")

        # Declare the tf_prefix parameter
        self.declare_parameter("tf_prefix", "")
        self.tf_prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value

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
        self.origin_set = True
        self.orig_heading_set = False

        # time between imu updates, sec
        self.dt_gps = 1 / self.freq

        # our graph object, for reference frame
        self.graph = get_coordinate_transfer()

        # Hardcoded initial datum
        self.lat = 43.06999991995453
        self.lon = -89.40010098905695
        self.alt = 260.00
        self.graph.set_graph(self.lat, self.lon, self.alt)
        self.get_logger().info(f"Initial LTP datum set: lat={self.lat}, lon={self.lon}, alt={self.alt}")

        # subscribers
        self.sub_gps = self.create_subscription(
            NavSatFix, "/input/gps", self.gps_callback, 1
        )

        self.sub_mag = self.create_subscription(
            MagneticField, "/input/magnetometer", self.mag_callback, 1
        )

        # publishers
        self.pub_objects = self.create_publisher(
            VehicleState, "/output/filtered_state", 1
        )

        self.pub_initial_pose = self.create_publisher(
            PoseStamped, "artcar_1/initial_pose", 1
        )

        self.pub_odometry = self.create_publisher(
            Odometry, f"{self.tf_prefix}/odometry", 1  # Create the odometry publisher
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(1 / self.freq, self.pub_callback)
        self.timer_initial_pose = self.create_timer(1.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        if not self.origin_set:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.init_x
            pose.pose.position.y = self.init_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = math.cos(self.init_theta * 0.5)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(self.init_theta * 0.5)

            self.pub_initial_pose.publish(pose)

    def initial_pose_callback(self, msg):
        if not self.origin_set:
            self.origin_set = True
            self.init_x = msg.pose.position.x
            self.init_y = msg.pose.position.y
            self.init_theta = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
            self.graph.set_graph(self.init_x, self.init_y, 0)
            self.graph.set_rotation(self.init_theta)
            self.get_logger().info(f"Initial LTP datum updated: x={self.init_x}, y={self.init_y}, theta={self.init_theta}")

    # CALLBACKS:
    def mag_callback(self, msg):
        """Callback for the Magnetometer subscriber.

        Read the Magnetometer observation from the topic. Process this into a heading angle, and then set the rotation of the LTP if the original heading has not been set yet.

        Args:
            msg: The message received from the topic
        """
        self.mag = msg
        mag_x = self.mag.magnetic_field.x
        mag_y = self.mag.magnetic_field.y
        mag_z = self.mag.magnetic_field.z
        xGauss = mag_x * 0.48828125
        yGauss = mag_y * 0.48828125
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
            # self.graph.set_rotation(np.deg2rad(self.D) - self.init_theta)
            # Does changing this set LTP to ENU?
            self.graph.set_rotation(self.init_theta)

            self.state[2, 0] = np.deg2rad(self.D) #self.init_theta
            self.get_logger().info(f"Initial heading set: theta={np.deg2rad(self.D)}")

    def gps_callback(self, msg):
        """Callback for the GPS subscriber.

        Read the GPS observation from the topic. If the original heading has been set, initialize the LTP and set this point as the origin. If the origin has been set, project this gps coordinate onto the defined LTP using the graph object, and return that x and y coordinate.

        Args:
            msg: The message received from the topic
        """
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
        """Callback for the publisher.

        Publish the estimated state to the `filtered_state` topic. This filter just directly passes the LTP x and y coordinates as recorded by the GPS, the heading as recorded by the Magnetometer, and the change in position since the last GPS measurement.
        """
        msg = VehicleState()
        msg.pose.position.x = float(self.x)
        msg.pose.position.y = float(self.y)

        # Calculate quaternion for yaw rotation
        yaw = np.deg2rad(self.D)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        # Set the orientation quaternion
        msg.pose.orientation.w = cy
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = sy

        msg.twist.linear.x = float(self.gtvx)
        msg.twist.linear.y = float(self.gtvy)

        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_objects.publish(msg)

        # Publish the transform from map to odom
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.tf_prefix + '/odom'

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = cy
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy

        self.tf_broadcaster.sendTransform(t)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = self.tf_prefix + "/base_link"

        # Set position
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy

        # Set velocity
        odom.twist.twist.linear.x = float(self.gtvx)
        odom.twist.twist.linear.y = float(self.gtvy)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0  # Assuming no angular velocity for simplicity

        self.pub_odometry.publish(odom)


def main(args=None):
    print("=== Starting State Estimation Node ===")
    rclpy.init(args=args)
    estimator = GroundTruthNode()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
