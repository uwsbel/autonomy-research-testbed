import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # create subscribers
        self.accelerometer_sub = self.create_subscription(Imu,'/chrono_ros_bridge/output/accelerometer/data',self.accelerometer_callback,10)
        # self.magnetometer_sub = self.create_subscription(MagneticField,'/chrono_ros_bridge/output/magnetometer/data',self.imu_callback,10)
        self.gyroscope_sub = self.create_subscription(Imu,'/chrono_ros_bridge/output/gyroscope/data',self.gyroscope_callback,10)

        self.angular_velocity = None
        self.linear_acceleration = None
        self.orientation = None
        # self.magnetic_field = None

        # create publishers
        self.publisher = self.create_publisher(Imu, '/imu0', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def accelerometer_callback(self, msg):
        self.linear_acceleration = msg.linear_acceleration
        self.orientation = msg.orientation
    
    def gyroscope_callback(self, msg):
        self.angular_velocity = msg.angular_velocity

    def timer_callback(self):
        if self.angular_velocity is not None and self.linear_acceleration is not None and self.orientation is not None:
            # build msg
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.orientation = self.orientation
            msg.angular_velocity = self.angular_velocity
            msg.linear_acceleration = self.linear_acceleration

            # publish
            self.publisher.publish(msg)
            # self.get_logger().info('Publishing: "%s"' % msg)

            # reset
            self.orientation = None
            self.angular_velocity = None
            self.linear_acceleration = None



def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()