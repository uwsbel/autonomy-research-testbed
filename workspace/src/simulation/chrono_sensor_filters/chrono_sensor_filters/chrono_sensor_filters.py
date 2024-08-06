import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, NavSatFix
from rclpy.qos import qos_profile_sensor_data
import math
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from art_msgs.msg import VehicleInput as VehicleInput
from chrono_ros_interfaces.msg import DriverInputs as ChVehicleInput
from builtin_interfaces.msg import Time as BuiltinTime
from rosgraph_msgs.msg import Clock

class ChronoSensorFilters(Node):

    '''
            General class to transform Chrono IMU data to REP-145 IMU data... also it's a mess. 
                        
            Originally published ~/imu/data_raw but using imu_filter_madgwick feels wasteful. Computes its own heading for now, yoinked the code from ground_truth.

            The point of this class is that the interface point between sim localization & reality localization should be exactly the same:

            Sim: /output/* => ~/imu/data, ~/gps/fix
            Reality: ~/imu/data, ~/gps/fix

            Also solves discrepency with VehicleInput driver topics
            Rover SW: ~/control/vehicle_inputs (art_msgs VehicleInputs)
            SIM: ~/input/driver_inputs (chrono_ros_interfaces DriverInputs)
    '''
    
    def __init__(self):
        super().__init__('chrono_sensor_filters')
        self.imu_publisher = self.create_publisher(Imu, '/output/imu/data', qos_profile_sensor_data)
        self.gps_publisher = self.create_publisher(NavSatFix, '/output/gps/fix', qos_profile_sensor_data)
        # self.clock_publisher = self.create_publisher(Clock, '/clock', qos_profile_sensor_data)

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.control_publisher = self.create_publisher(ChVehicleInput, '/output/vehicle/driver_inputs', qos_profile)

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

        self.control_subscription = self.create_subscription(
            VehicleInput,
            '/input/control/vehicle_inputs',
            self.control_callback,
            qos_profile_sensor_data
        )

        self.gyro_data = None
        self.accel_data = None
        self.mag_data = None
        self.heading = 0.0

    def gyro_callback(self, msg):
        self.gyro_data = msg
        self.publish_combined_imu()

    def accel_callback(self, msg):
        self.accel_data = msg
        self.publish_combined_imu()

    def gps_callback(self, msg):
        msg.header.frame_id = f'{self.tf_prefix}/gps'
        msg.position_covariance = [0.0] * 9  
        self.gps_publisher.publish(msg)

        
    def mag_callback(self, msg):
        self.mag_data = msg
        mag_x = self.mag_data.magnetic_field.x
        mag_y = self.mag_data.magnetic_field.y

        xGauss = mag_x * 0.48828125
        yGauss = mag_y * 0.48828125  

        if xGauss == 0:
            self.heading = 0 if yGauss < 0 else 90
        else:
            self.heading = math.atan2(yGauss, xGauss) * 180 / math.pi

        self.heading -= 16.1
        while self.heading > 360:
            self.heading -= 360
        while self.heading < 0:
            self.heading += 360

    def control_callback(self, msg):
        veh_input = ChVehicleInput()
        veh_input.throttle = msg.throttle
        veh_input.steering = msg.steering
        veh_input.braking = msg.braking
        self.control_publisher.publish(veh_input)

    def heading_to_quaternion(self, heading):
        yaw = np.deg2rad(heading)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def publish_combined_imu(self):
        if self.gyro_data and self.accel_data:
            combined_imu = Imu()
            combined_imu.header.stamp = self.accel_data.header.stamp #self.get_clock().now().to_msg()
            combined_imu.header.frame_id =  f'{self.tf_prefix}/base_link'

            combined_imu.angular_velocity = self.gyro_data.angular_velocity
            combined_imu.angular_velocity_covariance = self.gyro_data.angular_velocity_covariance

            combined_imu.linear_acceleration = self.accel_data.linear_acceleration
            combined_imu.linear_acceleration_covariance = self.accel_data.linear_acceleration_covariance

            quaternion = self.heading_to_quaternion(self.heading)
            combined_imu.orientation.x = quaternion[0]
            combined_imu.orientation.y = quaternion[1]
            combined_imu.orientation.z = quaternion[2]
            combined_imu.orientation.w = quaternion[3]

            self.imu_publisher.publish(combined_imu)

            # Publish GPS time to /clock topic
            # gps_time = Clock()
            # gps_time.clock = BuiltinTime(sec=combined_imu.header.stamp.sec, nanosec=combined_imu.header.stamp.nanosec)
            # self.clock_publisher.publish(gps_time)


def main(args=None):
    rclpy.init(args=args)
    chrono_sensor_filters = ChronoSensorFilters()
    rclpy.spin(chrono_sensor_filters)
    chrono_sensor_filters.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
