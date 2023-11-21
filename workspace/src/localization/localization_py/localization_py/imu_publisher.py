import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3

class IMUAggregatorNode(Node):
    def __init__(self):
        super().__init__('imu_aggregator_node')

        # Subscribers for Vector3 type
        # self.sub_accelerometer_vector3 = self.create_subscription(
        #     Vector3, '/sensing/accelerometer/data', self.accelerometer_vector3_callback, 10)
        # Subscribers for Imu type
        self.sub_accelerometer_imu = self.create_subscription(
            Imu, '/sensing/accelerometer/data', self.accelerometer_imu_callback, 10)

        self.sub_gyroscope = self.create_subscription(
            Vector3, '/sensor/gyroscope/data', self.gyroscope_callback, 10)
        self.sub_magnetometer = self.create_subscription(
            MagneticField, '/sensing/magnetometer/data', self.magnetometer_callback, 10)

        # Publisher
        self.pub_imu = self.create_publisher(Imu, '/sensing/imu/data', 10)

        # IMU data
        self.imu_data = Imu()
        self.imu_data.header.frame_id = 'imu_link'  # Adjust the frame_id as needed

    def accelerometer_vector3_callback(self, msg):
        self.imu_data.linear_acceleration.x = msg.x
        self.imu_data.linear_acceleration.y = msg.y
        self.imu_data.linear_acceleration.z = msg.z
        self.publish_imu_data()

    def accelerometer_imu_callback(self, msg):
        self.imu_data.linear_acceleration = msg.linear_acceleration
        self.publish_imu_data()

    def gyroscope_callback(self, msg):
        self.imu_data.angular_velocity.x = msg.x
        self.imu_data.angular_velocity.y = msg.y
        self.imu_data.angular_velocity.z = msg.z
        self.publish_imu_data()

    def magnetometer_callback(self, msg):
        # Handle magnetometer data if necessary
        pass

    def publish_imu_data(self):
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu.publish(self.imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_aggregator_node = IMUAggregatorNode()
    rclpy.spin(imu_aggregator_node)
    imu_aggregator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
