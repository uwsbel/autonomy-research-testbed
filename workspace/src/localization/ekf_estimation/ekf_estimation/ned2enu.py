import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import tf_transformations
class IMUConverterNode(Node):
    def __init__(self):
        super().__init__('imu_converter_node')
        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.get_logger().info('IMU Converter Node has been started.')

    def imu_callback(self, msg):
        converted_msg = self.convert_ned_to_enu(msg)
        self.publisher.publish(converted_msg)

    def convert_ned_to_enu(self, msg):
        # Create a new IMU message for the converted data
        converted_msg = Imu()

        # Copy the header
        converted_msg.header = msg.header

        # Convert linear acceleration
        converted_msg.linear_acceleration.x = msg.linear_acceleration.y
        converted_msg.linear_acceleration.y = msg.linear_acceleration.x
        converted_msg.linear_acceleration.z = -msg.linear_acceleration.z

        # Convert angular velocity
        converted_msg.angular_velocity.x = msg.angular_velocity.y
        converted_msg.angular_velocity.y = msg.angular_velocity.x
        converted_msg.angular_velocity.z = -msg.angular_velocity.z

        # Convert orientation (quaternion)
        q_ned = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        #q_enu = self.quaternion_ned_to_enu(q_ned)
        q_enu_with_offset = self.add_90_degree_offset(q_ned)
        converted_msg.orientation.x = q_enu_with_offset[0]
        converted_msg.orientation.y = q_enu_with_offset[1]
        converted_msg.orientation.z = q_enu_with_offset[2]
        converted_msg.orientation.w = q_enu_with_offset[3]

        return converted_msg

    def quaternion_ned_to_enu(self, q_ned):
        # Convert NED to ENU
        q_ned_to_enu = np.array([0, 1, 1, -1])
        q_enu = np.zeros(4)
        q_enu[0] = q_ned[1]
        q_enu[1] = q_ned[0]
        q_enu[2] = -q_ned[2]
        q_enu[3] = q_ned[3]
        return q_enu

    def add_90_degree_offset(self, q):
        q_wxyz = [q[0],q[1],q[2],q[3]]

        yaw_angle = np.deg2rad(-140.)
        yaw_quaternion = tf_transformations.quaternion_about_axis(yaw_angle, (0, 0, 1))
        
        q_prime = tf_transformations.quaternion_multiply(yaw_quaternion, q_wxyz)

        return np.array([q_prime[0],q_prime[1],q_prime[2],q_prime[3]])

def main(args=None):
    rclpy.init(args=args)
    node = IMUConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

