import sys
import threading
import math
import time
from argparse import ArgumentParser, Namespace
from typing import List, Optional

import pytermgui as ptg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.pubs = {
            "artcar_1": self.create_publisher(Twist, '/artcar_1/cmd_vel', 10),
            "artcar_2": self.create_publisher(Twist, '/artcar_2/cmd_vel', 10),
            "artcar_3": self.create_publisher(Twist, '/artcar_3/cmd_vel', 10)
        }

        self.qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.namespaces = list(self.pubs.keys())
        self.current_namespace_index = 0
        self.current_namespace = self.namespaces[self.current_namespace_index]
        self.twist = Twist()

        self.odom_data = None
        self.gps_data = None
        self.imu_data = None

        self.subscribers = {}

        self.create_subscriptions(self.current_namespace)

    def create_subscriptions(self, namespace):
        if "odom" in self.subscribers:
            self.destroy_subscription(self.subscribers["odom"])
        if "gps" in self.subscribers:
            self.destroy_subscription(self.subscribers["gps"])
        if "imu" in self.subscribers:
            self.destroy_subscription(self.subscribers["imu"])

        self.subscribers["odom"] = self.create_subscription(
            Odometry, f'/{namespace}/odometry/filtered', self.odom_callback, self.qos_profile)
        self.subscribers["gps"] = self.create_subscription(
            NavSatFix, f'/{namespace}/gps/fix', self.gps_callback, self.qos_profile)
        self.subscribers["imu"] = self.create_subscription(
            Imu, f'/{namespace}/imu/data', self.imu_callback, self.qos_profile)
            
        self.odom_data = None
        self.gps_data = None
        self.imu_data = None

    def odom_callback(self, msg: Odometry):
        self.odom_data = msg

    def gps_callback(self, msg: NavSatFix):
        self.gps_data = msg

    def imu_callback(self, msg: Imu):
        self.imu_data = msg

    def get_odometry_data(self) -> str:
        if self.odom_data:
            position = self.odom_data.pose.pose.position
            orientation = self.odom_data.pose.pose.orientation
            heading = self.get_heading(orientation)
            covariance = self.odom_data.pose.covariance
            avg_variance = sum(covariance[i] for i in range(0, 36, 7)) / 6  # Average of the diagonal elements
            return (f"Position: ({position.x:.2f}, {position.y:.2f})\n"
                    f"Heading: {heading:.2f} degrees (ENU)\n"
                    f"Avg. Variance: {avg_variance:.2e}")
        return "---"

    def get_gps_data(self) -> str:
        if self.gps_data:
            covariance = self.gps_data.position_covariance
            avg_variance = sum(covariance[i] for i in range(0, 9, 4)) / 3  # Average of the diagonal elements
            return (f"Latitude: {self.gps_data.latitude:.6f}\n"
                    f"Longitude: {self.gps_data.longitude:.6f}\n"
                    f"Altitude: {self.gps_data.altitude:.2f} m\n"
                    f"Var: {avg_variance:.2e}")
        return "---"

    def get_imu_data(self) -> str:
        if self.imu_data:
            orientation = self.imu_data.orientation
            acceleration = self.imu_data.linear_acceleration
            angular_velocity = self.imu_data.angular_velocity
            orientation_covariance = self.imu_data.orientation_covariance
            linear_acceleration_covariance = self.imu_data.linear_acceleration_covariance
            angular_velocity_covariance = self.imu_data.angular_velocity_covariance
            avg_orientation_variance = sum(orientation_covariance[i] for i in range(0, 9, 4)) / 3
            avg_linear_acceleration_variance = sum(linear_acceleration_covariance[i] for i in range(0, 9, 4)) / 3
            avg_angular_velocity_variance = sum(angular_velocity_covariance[i] for i in range(0, 9, 4)) / 3
            return (f"Orientation: ({orientation.x:.2f}, {orientation.y:.2f}, {orientation.z:.2f}, {orientation.w:.2f})\n"
                    f"Acceleration: ({acceleration.x:.2f}, {acceleration.y:.2f}, {acceleration.z:.2f}) m/s²\n"
                    f"Angular Velocity: ({angular_velocity.x:.2f}, {angular_velocity.y:.2f}, {angular_velocity.z:.2f}) rad/s\n"
                    f"Ang Var: {avg_orientation_variance:.2e}\n"
                    f"LinAccel Var: {avg_linear_acceleration_variance:.2e}\n"
                    f"AngVel Variance: {avg_angular_velocity_variance:.2e}")
        return "---"

    def get_heading(self, orientation):
        # Convert quaternion to Euler angles
        q = orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        heading_degrees = math.degrees(heading)
        # Clamp heading between 0 and 360
        heading_degrees = heading_degrees % 360
        return heading_degrees

    def set_throttle(self, increment: float):
        self.twist.linear.x = max(-1.0, min(1.0, self.twist.linear.x + increment))
        self.publish_cmd()

    def set_steering(self, increment: float):
        self.twist.angular.z = max(-1.0, min(1.0, self.twist.angular.z + increment))
        self.publish_cmd()

    def publish_cmd(self):
        self.pubs[self.current_namespace].publish(self.twist)

    def toggle_namespace_forward(self):
        self.current_namespace_index = (self.current_namespace_index + 1) % len(self.namespaces)
        self.current_namespace = self.namespaces[self.current_namespace_index]
        self.create_subscriptions(self.current_namespace)

    def toggle_namespace_backward(self):
        self.current_namespace_index = (self.current_namespace_index - 1) % len(self.namespaces)
        self.current_namespace = self.namespaces[self.current_namespace_index]
        self.create_subscriptions(self.current_namespace)

def _process_arguments(argv: Optional[List[str]] = None) -> Namespace:
    parser = ArgumentParser(description="My first PTG application.")
    return parser.parse_args(argv)

def _create_aliases() -> None:
    ptg.tim.alias("app.header", "@157 240")
    ptg.tim.alias("app.footer", "inverse app.header")
    ptg.tim.alias("app.highlight", "bold inverse")
    ptg.tim.alias("app.error", "@red inverse")
    ptg.tim.alias("app.footer.inverse.gray", "inverse gray")

def _configure_widgets() -> None:
    ptg.boxes.EMPTY.set_chars_of(ptg.Window)
    ptg.Splitter.set_char("separator", "")

def _define_layout() -> ptg.Layout:
    layout = ptg.Layout()
    layout.add_slot("header", height=1)
    layout.add_break()
    layout.add_slot("namespace_display", height=2)
    layout.add_break()
    layout.add_slot("body", width=0.5)
    layout.add_slot("telemetry", width=0.5)
    layout.add_break()
    layout.add_slot("footer", height=2)
    return layout

def macro_ros_time(fmt: str, teleop_node: TeleopNode) -> str:
    now = teleop_node.get_clock().now()
    current_time = now.to_msg()
    formatted_time = time.strftime(fmt, time.localtime(current_time.sec))
    return formatted_time

def macro_odometry_data(teleop_node: TeleopNode) -> str:
    return teleop_node.get_odometry_data()

def macro_gps_data(teleop_node: TeleopNode) -> str:
    return teleop_node.get_gps_data()

def macro_imu_data(teleop_node: TeleopNode) -> str:
    return teleop_node.get_imu_data()

def ros_spin_thread(teleop_node):
    while rclpy.ok():
        rclpy.spin_once(teleop_node, timeout_sec=1.0)

def main(argv: Optional[List[str]] = None) -> None:
    _create_aliases()
    _configure_widgets()
    args = _process_arguments(argv)

    rclpy.init(args=argv)

    teleop_node = TeleopNode()

    ptg.tim.define("!rostime", lambda fmt: macro_ros_time(fmt, teleop_node))
    ptg.tim.define("!odometry", lambda _: macro_odometry_data(teleop_node))
    ptg.tim.define("!gps", lambda _: macro_gps_data(teleop_node))
    ptg.tim.define("!imu", lambda _: macro_imu_data(teleop_node))

    # Start the ROS spin loop in a separate thread
    threading.Thread(target=ros_spin_thread, args=(teleop_node,)).start()

    custom_box = ptg.boxes.Box(["---", "x", ""])

    with ptg.WindowManager() as manager:

        manager.layout = _define_layout()

        header = ptg.Window("[app.header] ART Dashboard ", box="EMPTY")
        manager.add(header, assign="header")

        footer = ptg.Window(ptg.Button("Quit", lambda *_: manager.stop()), box="EMPTY")
        manager.add(footer, assign="footer")

        namespace_label = ptg.Label(f"Current Namespace: <{teleop_node.current_namespace}>")
        namespace_display = ptg.Window(namespace_label, box=custom_box)
        manager.add(namespace_display, assign="namespace_display")

        body_window = ptg.Window("Use arrow keys for teleoperation", 
                                 "Up/Down: Throttle", "Left/Right: Steering", 
                                 box=custom_box, draggable=False)
        
        throttle_widget = ptg.Label(f"Throttle: {teleop_node.twist.linear.x:.1f}")
        steering_widget = ptg.Label(f"Steering: {teleop_node.twist.angular.z:.1f}")
        body_window += throttle_widget
        body_window += steering_widget
        
        manager.add(body_window, assign="body")

        telemetry_window = ptg.Window(
            ptg.Container("EKF Output", ptg.Label("[!odometry]%s")),
            ptg.Container("GPS Data", ptg.Label("[!gps]%s")),
            ptg.Container("IMU Data", ptg.Label("[!imu]%s")),
            box=custom_box,
            draggable=False,
            vertical_align=ptg.VerticalAlignment.TOP
        )
        
        manager.add(telemetry_window, assign="telemetry")

        time_label = ptg.Label("ROS Time: [!rostime]%c")
        time_display = ptg.Window(time_label, box=custom_box, draggable=False)
        manager.add(time_display, assign="footer")

        def update_namespace_label(highlight: bool = False):
            if highlight:
                namespace_label.value = f"Current Namespace: [app.highlight]<{teleop_node.current_namespace}>[/app.highlight]"
            else:
                namespace_label.value = f"Current Namespace: <{teleop_node.current_namespace}>"

        def handle_key(self, key: str):
            if key == ptg.keys.UP:
                teleop_node.set_throttle(0.1)
            elif key == ptg.keys.DOWN:
                teleop_node.set_throttle(-0.1)
            elif key == ptg.keys.LEFT:
                teleop_node.set_steering(0.1)
            elif key == ptg.keys.RIGHT:
                teleop_node.set_steering(-0.1)
            elif key == 'b':
                teleop_node.toggle_namespace_forward()
                update_namespace_label(highlight=True)
                threading.Timer(1.0, update_namespace_label).start()
            elif key == 'v':
                teleop_node.toggle_namespace_backward()
                update_namespace_label(highlight=True)
                threading.Timer(1.0, update_namespace_label).start()
            elif key == 'esc':
                teleop_node.set_throttle(0.0)
                teleop_node.set_steering(0.0)
                manager.stop()
            throttle_widget.value = f"Throttle: {teleop_node.twist.linear.x:.1f}"
            steering_widget.value = f"Steering: {teleop_node.twist.angular.z:.1f}"

        manager.bind(ptg.keys.UP, handle_key)
        manager.bind(ptg.keys.LEFT, handle_key)
        manager.bind(ptg.keys.RIGHT, handle_key)
        manager.bind(ptg.keys.DOWN, handle_key)
        manager.bind("b", handle_key)
        manager.bind("v", handle_key)
        manager.bind("esc", handle_key)

    teleop_node.destroy_node()
    rclpy.shutdown()
    ptg.tim.print("[!gradient(210)]Goodbye!")

if __name__ == "__main__":
    main(sys.argv[1:])

