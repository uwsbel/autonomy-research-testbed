import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('IMU Publisher Node Started')

    def timer_callback(self):
        msg = Imu()
        msg.angular_velocity.x = random.uniform(-1, 1)
        msg.angular_velocity.y = random.uniform(-1, 1)
        msg.angular_velocity.z = random.uniform(-1, 1)
        msg.linear_acceleration.x = random.uniform(-9.8, 9.8)
        msg.linear_acceleration.y = random.uniform(-9.8, 9.8)
        msg.linear_acceleration.z = random.uniform(-9.8, 9.8)
        msg.orientation.w = math.sqrt(1 - msg.orientation.x**2 - msg.orientation.y**2 - msg.orientation.z**2)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
