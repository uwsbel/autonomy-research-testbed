import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from art_msgs.msg import VehicleInput
from std_msgs.msg import Header
import math
import time

class CalibrateHeadingNode(Node):
    def __init__(self, namespace):
        super().__init__('calibrate_heading')
        self.get_logger().info("Initiated calibration node")

        # Parameters (hard-coded for simplicity)
        self.x_vel = 1.0
        self.x_vel_time = 10.0 

        # Initialize subscribers and publishers with namespace
        self.sub_odom = self.create_subscription(Odometry, f'/{namespace}/odometry/filtered', self.filtered_odom_CB, 100)
        self.pubVehicleInputs = self.create_publisher(VehicleInput, f'/{namespace}/control/vehicle_inputs', 100)

        self.get_logger().warn(f"PLEASE ENSURE YOU HAVE MIN. {self.x_vel * self.x_vel_time:.1f} m OF CLEAR SPACE IN FRONT OF YOUR ROBOT FOR CALIBRATION")

        # Set publish rate and calculate number of messages to count
        self.pubRate = 10
        self.numVelMsgs = int(self.x_vel_time * self.pubRate)

        # Start calibration
        self.calibrate_heading()

    def filtered_odom_CB(self, odom_msgs):
        self.y_pos = odom_msgs.pose.pose.position.y
        self.x_pos = odom_msgs.pose.pose.position.x

    def calibrate_heading(self):
        rate = self.create_rate(self.pubRate)
        vehicle_input_msg = VehicleInput()
        vehicle_input_msg.header = Header()
        vehicle_input_msg.steering = 0.0
        vehicle_input_msg.throttle =0.8 
        vehicle_input_msg.braking = 0.0

        self.get_logger().info("Start")

        # Move forward
        for _ in range(self.numVelMsgs):
            vehicle_input_msg.header.stamp = self.get_clock().now().to_msg()
            self.pubVehicleInputs.publish(vehicle_input_msg)
            time.sleep(0.1)

        time.sleep(2)  # Pause for 2 seconds to prevent quick forwards and backwards movement
        self.get_logger().info("Stop")


        self.y_pos = None
        self.x_pos = None

        def wait_for_odom():
            rclpy.spin_once(self)
            return self.y_pos is not None and self.x_pos is not None

        while not wait_for_odom():
            self.get_logger().info("Waiting for odometry message...")

        heading_error = math.atan2(self.y_pos, self.x_pos)
        self.get_logger().info(f"Detected heading error of: {heading_error * 180 / math.pi:.1f} Degrees")

        # Move back to the start position
        self.get_logger().info("Returning to start...")
        vehicle_input_msg.throttle = 0.0
        vehicle_input_msg.braking = 1.0
        for _ in range(self.numVelMsgs):
            vehicle_input_msg.header.stamp = self.get_clock().now().to_msg()
            self.pubVehicleInputs.publish(vehicle_input_msg)
            rate.sleep()

        self.get_logger().info("Heading Calibration Complete")

        self.get_logger().info("Ending Node...")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    namespace = 'artcar_1'  # Default namespace if none is provided

    if args is not None:
        if len(args) > 1:
            namespace = args[1]

    node = CalibrateHeadingNode(namespace)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

