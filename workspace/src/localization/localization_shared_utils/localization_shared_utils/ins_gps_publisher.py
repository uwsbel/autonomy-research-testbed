import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

class SensorAggregatorNode(Node):
    def __init__(self):
        super().__init__('sensor_aggregator_node')
        # IMU Publisher
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)

        # Dictionary to hold publishers for each GPS fix topic
        self.gps_publishers = {}

        # List of GPS base topics for which to create publishers
        gps_base_topics = ['/fix1', '/fix2', '/fix3']
        gps_suffixes = ['', '_airsim', '_airsim_default']

        # Initialize publishers for each GPS fix topic
        for base_topic in gps_base_topics:
            for suffix in gps_suffixes:
                fix_topic = base_topic + suffix
                self.gps_publishers[fix_topic] = self.create_publisher(NavSatFix, fix_topic, 10)

        # IMU data initialization
        self.imu_data = Imu()
        self.imu_data.header.frame_id = 'imu'

        # Timestamp for IMU rate control
        self.last_imu_publish_time = self.get_clock().now()
        self.imu_publish_interval = 0.1  # 10 Hz interval

        # Dictionary to hold the last publish time for each GPS topic to control publish rate
        self.last_gps_publish_time = {topic: self.get_clock().now() for topic in self.gps_publishers}

        # GPS publish interval in seconds (10 Hz)
        self.gps_publish_interval = 0.1

        # IMU Subscriptions
        self.sub_accelerometer_imu = self.create_subscription(
            Imu, '/chrono_ros_node/output/accelerometer/data', self.accelerometer_imu_callback, 10)
        self.sub_gyroscope = self.create_subscription(
            Imu, '/chrono_ros_node/output/gyroscope/data', self.gyroscope_callback, 10)

        # Subscribe to GPS data topics and bind them to corresponding output topics
        for i, base_topic in enumerate(['/chrono_ros_node/output/gps/gps1', '/chrono_ros_node/output/gps/gps2', '/chrono_ros_node/output/gps/gps3'], start=1):
            for suffix in gps_suffixes:
                input_topic = f"{base_topic}{suffix}"
                output_topic = f"/fix{i}{suffix}"
                self.create_subscription(NavSatFix, input_topic, lambda msg, topic=output_topic: self.gps_callback(msg, topic), 10)

    def accelerometer_imu_callback(self, msg):
        # Update IMU acceleration data
        self.imu_data.linear_acceleration = msg.linear_acceleration
        self.publish_imu_data()

    def gyroscope_callback(self, msg):
        # Update IMU gyroscope data
        self.imu_data.angular_velocity = msg.angular_velocity
        self.publish_imu_data()

    def publish_imu_data(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_imu_publish_time).nanoseconds / 1e9 >= self.imu_publish_interval:
            self.imu_data.header.stamp = current_time.to_msg()
            self.pub_imu.publish(self.imu_data)
            self.last_imu_publish_time = current_time

    def gps_callback(self, msg, topic):
        # Get the current time
        current_time = self.get_clock().now()

        # Check if it's time to publish based on the interval
        if (current_time - self.last_gps_publish_time[topic]).nanoseconds / 1e9 >= self.gps_publish_interval:
            # Publish the GPS data to the corresponding fix topic
            self.publish_gps_data(msg, topic)
            # Update the last publish time for this topic
            self.last_gps_publish_time[topic] = current_time

    def publish_gps_data(self, msg, topic):
        # Set the header timestamp and frame ID
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'

        # Publish the message using the appropriate publisher
        self.gps_publishers[topic].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
