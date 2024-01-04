import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix


class IMUAggregatorNode(Node):
    def __init__(self):
        super().__init__('imu_aggregator_node')

        # Subscribers for Vector3 type
        # self.sub_accelerometer_vector3 = self.create_subscription(
        #     Vector3, '/sensing/accelerometer/data', self.accelerometer_vector3_callback, 10)
        # Subscribers for Imu type
        self.sub_accelerometer_imu = self.create_subscription(
            Imu, '/chrono_ros_node/output/accelerometer/data', self.accelerometer_imu_callback, 10)

        self.sub_gyroscope = self.create_subscription(
            Imu, '/chrono_ros_node/output/gyroscope/data', self.gyroscope_callback, 10)
        self.sub_magnetometer = self.create_subscription(
            MagneticField, '/chrono_ros_node/output/magnetometer/data', self.magnetometer_callback, 10)

        self.sub_gps = self.create_subscription(
            NavSatFix, '/chrono_ros_node/output/gps/data', self.gps_callback, 10)


        # Publisher
        self.pub_imu = self.create_publisher(Imu, '/imu', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/fix', 10)

        # IMU data
        self.imu_data = Imu()
        self.imu_data.header.frame_id = 'imu'  # Adjust the frame_id as needed
        self.gps_data = NavSatFix()
        self.gps_data.header.frame_id = 'gps' 
        self.timer = self.create_timer(0.2, self.publish_data) 


    # def accelerometer_vector3_callback(self, msg):
    #     self.imu_data.linear_acceleration.x = msg.x
    #     self.imu_data.linear_acceleration.y = msg.y
    #     self.imu_data.linear_acceleration.z = msg.z
    #     self.publish_data()

    def accelerometer_imu_callback(self, msg):
        self.imu_data.linear_acceleration = msg.linear_acceleration
        self.publish_data()

    def gyroscope_callback(self, msg):
        self.imu_data.angular_velocity = msg.angular_velocity
        self.publish_data()

    def magnetometer_callback(self, msg):
        pass
        # self.imu_data.magnetic_field.y = msg.magnetic_field.y
        # self.imu_data.magnetic_field.z = msg.magnetic_field.z
        # self.publish_imu_data()
        
    def gps_callback(self, msg):
        self.gps_data.latitude = msg.latitude
        self.gps_data.longitude = msg.longitude
        self.gps_data.altitude = msg.altitude
        self.gps_data.status = msg.status
        self.gps_data.position_covariance = msg.position_covariance
        self.gps_data.position_covariance_type = msg.position_covariance_type
        self.publish_data()
        

    def publish_imu_data(self):
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu.publish(self.imu_data)

    def publish_data(self):
    # Publish IMU data
        self.imu_data.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu.publish(self.imu_data)

        # Publish GPS data
        self.gps_data.header.stamp = self.get_clock().now().to_msg()
        self.pub_gps.publish(self.gps_data)

def main(args=None):
    rclpy.init(args=args)
    imu_aggregator_node = IMUAggregatorNode()
    rclpy.spin(imu_aggregator_node)
    imu_aggregator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Imu, MagneticField, NavSatFix
# from geometry_msgs.msg import Vector3

# class IMUAggregatorNode(Node):
#     def __init__(self):
#         super().__init__('imu_aggregator_node')

#         # Subscribers (note: these are still subscribed but won't affect the published data)
#         self.sub_accelerometer_imu = self.create_subscription(
#             Imu, '/chrono_ros_node/output/accelerometer/data', self.accelerometer_imu_callback, 10)
#         self.sub_gyroscope = self.create_subscription(
#             Vector3, '/chrono_ros_node/output/gyroscope/data', self.gyroscope_callback, 10)
#         self.sub_magnetometer = self.create_subscription(
#             MagneticField, '/chrono_ros_node/output/magnetometer/data', self.magnetometer_callback, 10)
#         self.sub_gps = self.create_subscription(
#             NavSatFix, '/chrono_ros_node/output/gps/data', self.gps_callback, 10)

#         # Publishers
#         self.pub_imu = self.create_publisher(Imu, '/imu', 10)
#         self.pub_gps = self.create_publisher(NavSatFix, '/fix', 10)

#         # Initialize IMU and GPS data with zeros
#         self.imu_data = Imu()
#         self.imu_data.header.frame_id = 'imu'
#         # self.imu_data.linear_acceleration.x = 0.0
#         # self.imu_data.linear_acceleration.y = 0.0
#         # self.imu_data.linear_acceleration.z = 0.0
#         # self.imu_data.angular_velocity.x = 0.0
#         # self.imu_data.angular_velocity.y = 0.0
#         # self.imu_data.angular_velocity.z = 0.0
#         # self.imu_data.orientation.x = 0.0
#         # self.imu_data.orientation.y = 0.0
#         # self.imu_data.orientation.z = 0.0
#         # self.imu_data.orientation.w = 1.0  # Quaternion identity

#         self.gps_data = NavSatFix()
#         self.gps_data.header.frame_id = 'gps'

#     def accelerometer_imu_callback(self, msg):
#         #self.imu_data.linear_acceleration = msg.linear_acceleration
#         self.publish_data()
#         pass

#     def gyroscope_callback(self, msg):
#         # self.imu_data.angular_velocity.x = msg.x
#         # self.imu_data.angular_velocity.y = msg.y
#         # self.imu_data.angular_velocity.z = msg.z
#         self.publish_data()
#         pass

#     def magnetometer_callback(self, msg):
#         # Do not update IMU data
#         self.publish_data()
#         pass

#     def gps_callback(self, msg):
#         self.gps_data.latitude = msg.latitude
#         self.gps_data.longitude = msg.longitude
#         self.gps_data.altitude = msg.altitude
#         self.gps_data.status = msg.status
#         self.gps_data.position_covariance = msg.position_covariance
#         self.gps_data.position_covariance_type = msg.position_covariance_type

#         self.publish_data()
#         pass

#     def publish_data(self):
#         current_time = self.get_clock().now().to_msg()
#         self.imu_data.header.stamp = current_time
#         self.gps_data.header.stamp = current_time
#         self.pub_imu.publish(self.imu_data)
#         self.pub_gps.publish(self.gps_data)

# def main(args=None):
#     rclpy.init(args=args)
#     imu_aggregator_node = IMUAggregatorNode()
#     rclpy.spin(imu_aggregator_node)
#     imu_aggregator_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
