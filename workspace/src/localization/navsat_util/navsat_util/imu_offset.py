import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import tf_transformations

class YawOffsetNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_offset')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.get_logger().info('IMU Offset Node has been started.')
        self.get_logger().info('##### ROVER MUST FACE EAST FOR CORRECT CALIBRATION #######.')
        self.get_logger().info('##### ####### ####### ####### ####### ####### ####### #######')

    def imu_callback(self, msg):
        current_yaw_deg = self.get_yaw_from_quaternion(msg.orientation)
        # Compute the offset to make the heading point East (0 degrees)
        yaw_offset_deg = -current_yaw_deg  # Offset needed to make the heading 0 degrees (East)
        converted_msg = self.add_offset(msg, yaw_offset_deg)
        self.publisher.publish(converted_msg)

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to Euler angles and return the yaw
        q_wxyz = [orientation.w, orientation.x, orientation.y, orientation.z]
        euler = tf_transformations.euler_from_quaternion(q_wxyz)
        yaw_rad = euler[2]
        yaw_deg = np.rad2deg(yaw_rad)
        return yaw_deg

    def add_offset(self, msg, yaw_offset_deg):
        q = msg.orientation
        q_wxyz = [q.w, q.x, q.y, q.z]

        yaw_angle = np.deg2rad(yaw_offset_deg)
        yaw_quaternion = tf_transformations.quaternion_about_axis(yaw_angle, (0, 0, 1))
        
        q_prime = tf_transformations.quaternion_multiply(yaw_quaternion, q_wxyz)

        msg.orientation.x = q_prime[1]
        msg.orientation.y = q_prime[2]
        msg.orientation.z = q_prime[3]
        msg.orientation.w = q_prime[0]
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = YawOffsetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
