#! /usr/bin/env python3

import threading
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np
import rclpy
from rclpy.subscription import Subscription
from rclpy.node import Node
from art_msgs.msg import VehicleInput
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy

class Example_Node(Node):
    """Example Node for plotting Throttle vs Acceleration using ROS 2 with best-effort QoS.
    
    Attributes:
        fig: Figure object for matplotlib
        ax: Axes object for matplotlib
        throttle_values: List of throttle values
        acceleration_values: List of corresponding acceleration values
        lock: lock for threading
        vehicle_input_sub: Subscriber for VehicleInput messages
        imu_data_sub: Subscriber for IMU data messages
    """

    def __init__(self):
        """Initialize."""
        super().__init__("example_node")
        # Initialize figure and axes for matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_title("Throttle vs Acceleration")
        self.ax.set_xlabel("Throttle")
        self.ax.set_ylabel("Acceleration")
        
        # Thread lock to prevent multi-access threading errors
        self._lock = threading.Lock()
        
        # Lists to store throttle and acceleration values
        self.throttle_values = []
        self.acceleration_values = []
        
        # Define QoS profile with best-effort reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribers for vehicle input and IMU data using best-effort QoS
        self.vehicle_input_sub: Subscription = self.create_subscription(
            VehicleInput, 
            "/artcar_1/control/vehicle_inputs", 
            self.vehicle_input_callback, 
            qos_profile
        )
        
        self.imu_data_sub: Subscription = self.create_subscription(
            Imu, 
            "/artcar_1/imu/data", 
            self.imu_data_callback, 
            qos_profile
        )
        
        # Temporary storage for latest IMU acceleration data
        self.latest_acceleration = 0.0

    def vehicle_input_callback(self, msg: VehicleInput):
        """Callback for VehicleInput messages.
        
        Args:
            msg: VehicleInput message
        """
        with self._lock:
            print(f"Received throttle value: {msg.throttle}")  # Debugging print
            # Store the throttle value and corresponding acceleration
            self.throttle_values.append(msg.throttle)
            self.acceleration_values.append(self.latest_acceleration)

    def imu_data_callback(self, msg: Imu):
        """Callback for IMU data messages.
        
        Args:
            msg: Imu message
        """
        with self._lock:
            # Calculate acceleration magnitude from IMU linear acceleration data
            linear_acc = msg.linear_acceleration
            self.latest_acceleration = np.sqrt(
                linear_acc.x**2 + linear_acc.y**2 + linear_acc.z**2
            )
            # print(f"Calculated acceleration: {self.latest_acceleration}")  # Debugging print

    def plt_func(self, _):
        """Function for updating the plot with new data.
        
        Args:
            _ : Dummy variable required for matplotlib animation.
        
        Returns:
            Axes object for matplotlib
        """
        with self._lock:
            print(f"####: {self.acceleration_values} #### {self.throttle_values}")  # Debugging print

            if len(self.throttle_values) > 0 and len(self.acceleration_values) > 0:
                # Only clear the plot when there are new data points
                self.ax.clear()
                self.ax.set_title("Throttle vs Acceleration")
                self.ax.set_xlabel("Throttle")
                self.ax.set_ylabel("Acceleration")
                self.ax.scatter(self.throttle_values, self.acceleration_values, color="blue")
                self.ax.grid(True)  # Add grid for better visualization
        return self.ax

    def _plt(self):
        """Function for initializing and showing matplotlib animation."""
        self.ani = anim.FuncAnimation(self.fig, self.plt_func, interval=2000)  # Increase interval to 2000 ms
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = Example_Node()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    node._plt()


if __name__ == "__main__":
    main()
