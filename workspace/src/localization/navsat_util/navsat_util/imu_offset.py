import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import tf_transformations

class YawOffsetNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_offset')
        self.declare_parameter('yaw_offset_deg', -140.0)
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.get_logger().info('IMU Offset Node has been started.')

    def imu_callback(self, msg):
        yaw_offset_deg = self.get_parameter('yaw_offset_deg').value
        converted_msg = self.add_offset(msg, yaw_offset_deg)
        self.publisher.publish(converted_msg)

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
