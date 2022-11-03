#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "opencv2/highgui.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"

std::string
mat_to_encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void frame_to_message(
    const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg) 
{
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_to_encoding(frame.type());
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  size_t depth = rmw_qos_profile_default.depth;
  double freq = 30.0;
  rmw_qos_reliability_policy_t reliability_policy = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy = rmw_qos_profile_default.history;


  std::string topic("image");
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto node = rclcpp::Node::make_shared("cam2image");
  rclcpp::Logger node_logger = node->get_logger();
  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;
  custom_camera_qos_profile.depth = depth;
  custom_camera_qos_profile.reliability = reliability_policy;
  custom_camera_qos_profile.history = history_policy;

  auto pub = node->create_publisher<sensor_msgs::msg::Image>(
      topic, custom_camera_qos_profile);

  bool is_flipped = false;

  auto callback =
    [&is_flipped, &node_logger](const std_msgs::msg::Bool::SharedPtr msg) -> void
    {
      is_flipped = msg->data;
      RCLCPP_INFO(node_logger, "Set flip mode to: %s", is_flipped ? "on" : "off");
    };

  rmw_qos_profile_t custom_flip_qos_profile = rmw_qos_profile_sensor_data;
  custom_flip_qos_profile.depth = 10;

  auto sub = node->create_subscription<std_msgs::msg::Bool>(
      "flip_image", callback, custom_flip_qos_profile);

  rclcpp::WallRate loop_rate(freq);

  cv::VideoCapture cap;
  cap.open(0);
  cv::Mat frame;
  cv::Mat flipped_frame;

  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->is_bigendian = false;
  size_t i = 1;

  while (rclcpp::ok()) {
    cap >> frame;
    if (!frame.empty()) {
      if (!is_flipped) {
        frame_to_message(frame, i, msg);
      } else {
        cv::flip(frame, flipped_frame, 1);
        frame_to_message(flipped_frame, i, msg);
      }
      pub->publish(msg);
      i++;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
