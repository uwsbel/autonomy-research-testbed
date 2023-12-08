import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

class LaserScanVisualizer(Node):
    def __init__(self):
        super().__init__('laser_scan_visualizer')
        self.subscription = self.create_subscription(
            LaserScan,
            '/chrono_ros_node/output/lidar_2d/data/laser_scan',  # Replace with the actual topic name
            self.laser_scan_callback,
            10  # Adjust the queue size as needed
        )
        self.laser_data = None
        self.ranges = []
        self.frame_number = 0

    def laser_scan_callback(self, msg):
        self.laser_data = msg
        self.ranges = np.array(msg.ranges)
        self.plot_laser_scan()
        #self.get_logger().info("Received laser scan data.")

    def plot_laser_scan(self):
        if self.ranges.size == 0:
            self.get_logger().info("No laser scan data received yet.")
            return

        angles = np.linspace(self.laser_data.angle_min, self.laser_data.angle_max, len(self.ranges))
        self.get_logger().info(f"Plotting laser scan data with {len(self.ranges)} points.")
        plt.figure()
        plt.plot(angles, self.ranges)
        plt.xlabel('Angle (radians)')
        plt.ylabel('Range (meters)')
        plt.title('Laser Scan Data')
        plt.grid(True)
        plt.show()
        # Save the plot with an incremental frame number
        # frame_filename = f'plot/figure_{self.frame_number}.png'
        # plt.savefig(frame_filename)
        # self.frame_number += 1

def main(args=None):
    rclpy.init(args=args)
    visualizer = LaserScanVisualizer()
    rclpy.spin(visualizer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
