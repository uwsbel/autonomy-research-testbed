import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np

class ImuFilterNode(Node):

    '''
            General class to transform Chrono IMU data to REP-145 IMU data... also it's a mess. 
            
            Change this to chrono_sensor_filters?
            
            Originally published ~/imu/data_raw but using imu_filter_madgwick feels wasteful. Computes its own heading for now, yoinked the code from ground_truth.

            The point of this class is that the interface point between sim localization & reality localization should be exactly the same:

            Sim: /output/* => ~/imu/data, ~/gps/fix
            Reality: ~/imu/data, ~/gps/fix
    '''

    def __init__(self):
        super().__init__('imu_filter')
        self.imu_publisher = self.create_publisher(Imu, '/output/imu/data', qos_profile_sensor_data)
        self.gps_publisher = self.create_publisher(NavSatFix, '/output/gps/fix', qos_profile_sensor_data)
        # self.magnetometer_publisher = self.create_publisher(MagneticField, '/output/magnetometer/data', qos_profile_sensor_data)
        
        self.declare_parameter('tf_prefix', 'artcar_1')
        self.tf_prefix = self.get_parameter('tf_prefix').get_parameter_value().string_value


        self.gyro_subscription = self.create_subscription(
            Imu,
            '/input/gyroscope/data',
            self.gyro_callback,
            qos_profile_sensor_data
        )

        self.accel_subscription = self.create_subscription(
            Imu,
            '/input/accelerometer/data',
            self.accel_callback,
            qos_profile_sensor_data
        )

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/input/gps/data',
            self.gps_callback,
            qos_profile_sensor_data
        )

        self.mag_subscription = self.create_subscription(
            MagneticField,
            '/input/magnetometer/data',
            self.mag_callback,
            qos_profile_sensor_data
        )

        # self.odom_subscription = self.create_subscription(
        #     Odometry,
        #     '/artcar_1/odometry/filtered',
        #     self.odom_callback,
        #     qos_profile_sensor_data
        # )

        self.gyro_data = None
        self.accel_data = None
        self.mag_data = None
        self.odom_data = None
        self.heading = 0.0

        # self.timer = self.create_timer(0.25, self.print_heading) 

    def gyro_callback(self, msg):
        self.gyro_data = msg
        self.publish_combined_imu()

    def accel_callback(self, msg):
        self.accel_data = msg
        self.publish_combined_imu()

    # Chrono noise model is weird, so just do this
    def gps_callback(self, msg):
        msg.header.frame_id = f'{self.tf_prefix}/gps'
        # Set covariance to zero before publishing
        msg.position_covariance = [0.0] * 9  
        self.gps_publisher.publish(msg)
        
    def mag_callback(self, msg):
        self.mag_data = msg
        # self.publish_magnetometer_data()

        # Calculate heading from magnetometer data
        mag_x = self.mag_data.magnetic_field.x
        mag_y = self.mag_data.magnetic_field.y

        # Conversion to Gauss if needed
        xGauss = mag_x * 0.48828125
        yGauss = mag_y * 0.48828125  

        if xGauss == 0:
            if yGauss < 0:
                self.heading = 0  
            else:
                self.heading = 90
        else:
            self.heading =  math.atan2(yGauss, xGauss) * 180 / math.pi # 

        self.heading -= 16.1
        # Normalize heading to 0-360 degrees
        while self.heading > 360:
            self.heading -= 360
        while self.heading < 0:
            self.heading += 360

        # Convert to ENU heading
        # self.heading = (450 - self.heading) % 360  # Adjusting for ENU frame

    def odom_callback(self, msg):
        self.odom_data = msg

    def heading_to_quaternion(self, heading):
        # Convert heading (yaw) to quaternion
        yaw = np.deg2rad(heading)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]  # [x, y, z, w]

    def publish_combined_imu(self):
        if self.gyro_data and self.accel_data:
            combined_imu = Imu()
            combined_imu.header.stamp = self.get_clock().now().to_msg()
            combined_imu.header.frame_id =  f'{self.tf_prefix}/base_link'

            # Copy gyroscope data
            combined_imu.angular_velocity = self.gyro_data.angular_velocity
            combined_imu.angular_velocity_covariance = self.gyro_data.angular_velocity_covariance

            # Copy accelerometer data
            combined_imu.linear_acceleration = self.accel_data.linear_acceleration
            combined_imu.linear_acceleration_covariance = self.accel_data.linear_acceleration_covariance

            # Convert heading to quaternion for orientation
            quaternion = self.heading_to_quaternion(self.heading)
            combined_imu.orientation.x = quaternion[0]
            combined_imu.orientation.y = quaternion[1]
            combined_imu.orientation.z = quaternion[2]
            combined_imu.orientation.w = quaternion[3]
            # combined_imu.orientation_covariance[0] = -1  # Indicating unknown covariance

            self.imu_publisher.publish(combined_imu)

    # def publish_magnetometer_data(self):
    #     if self.mag_data:
    #         mag_msg = MagneticField()
    #         mag_msg.header.stamp = self.get_clock().now().to_msg()
    #         mag_msg.header.frame_id = f'{self.tf_prefix}/imu'
    #         mag_msg.magnetic_field = self.mag_data.magnetic_field
    #         mag_msg.magnetic_field_covariance = self.mag_data.magnetic_field_covariance

    #         self.magnetometer_publisher.publish(mag_msg)

    def print_heading(self):
        if self.odom_data:
            # Extract quaternion from odometry message
            orientation_q = self.odom_data.pose.pose.orientation
            # Convert quaternion to Euler angles
            heading = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            self.get_logger().info(f'Odometry: {heading} degrees, Mag: {self.heading}')

    def quaternion_to_euler(self, x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * z + x * y)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t0, t1)
        # Convert radians to degrees
        heading = math.degrees(yaw)
        heading = heading % 360 if heading >= 0 else (heading + 360) % 360
        return heading

def main(args=None):
    rclpy.init(args=args)
    imu_combiner_node = ImuFilterNode()
    rclpy.spin(imu_combiner_node)
    imu_combiner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
