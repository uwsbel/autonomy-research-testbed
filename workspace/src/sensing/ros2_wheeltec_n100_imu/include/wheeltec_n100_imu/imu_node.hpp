#include <cstdio>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include "serial/serial.h"
#include "tf2/transform_datatypes.h"
 #include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>


#include "wheeltec_n100_imu/fdilink_data_struct.h"


// #include <rcl_interfaces>

using namespace std::chrono_literals;


#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN  0x38   //56
#define AHRS_LEN 0x30   //48
#define INSGPS_LEN 0x54 //84
#define PI 3.141592653589793
#define DEG_TO_RAD 0.017453292519943295
// Sample covariance
#define IMU_MAG_COV {0.01, 0.01, 0.01}
#define IMU_GYRO_COV {0.01, 0.01, 0.01}
#define IMU_ACCEL_COV {0.05, 0.05, 0.05}



