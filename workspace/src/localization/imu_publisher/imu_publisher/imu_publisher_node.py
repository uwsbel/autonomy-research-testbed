import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from rosgraph_msgs.msg import Clock


class IMUPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        # get parameters
        self.declare_parameter('sim_topics', False)
        self.sim_topics = self.get_parameter('sim_topics').value

        # create subscribers
        if self.sim_topics:
            self.accelerometer_sub = self.create_subscription(Imu,'/chrono_ros_bridge/output/accelerometer/data',self.accelerometer_callback,10)
            self.gyroscope_sub = self.create_subscription(Imu,'/chrono_ros_bridge/output/gyroscope/data',self.gyroscope_callback,10)
            self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        else:
            self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        if self.sim_topics:
            self.angular_velocity = None
            self.linear_acceleration = None
            self.orientation = None

        # create publishers
        self.pub = self.create_publisher(Imu, '/imu0', 10)
        
        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def accelerometer_callback(self, msg):
        self.linear_acceleration = msg.linear_acceleration
        self.orientation = msg.orientation
    

    def gyroscope_callback(self, msg):
        self.angular_velocity = msg.angular_velocity


    def clock_callback(self, msg):
        if self.angular_velocity is not None and self.linear_acceleration is not None and self.orientation is not None:
            # build msg
            imu_msg = Imu()
            imu_msg.header.stamp = msg.clock
            imu_msg.orientation = self.orientation
            imu_msg.angular_velocity = self.angular_velocity
            imu_msg.linear_acceleration = self.linear_acceleration

            # publish
            self.pub.publish(imu_msg)
            # self.get_logger().info('Publishing: "%s"' % msg)
    

    def imu_callback(self, msg):
        self.pub.publish(msg)


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