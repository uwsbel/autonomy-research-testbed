#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

class LidarTFPublisher : public rclcpp::Node
{
public:
  LidarTFPublisher()
    : Node("lidar_tf_publisher")
  {
    // Define the transformation
    transformStamped_.header.frame_id = "base_link";
    transformStamped_.child_frame_id = "unilidar_lidar";
    transformStamped_.transform.translation.x = 0.3;
    transformStamped_.transform.translation.y = 0.0;
    transformStamped_.transform.translation.z = 1.0;
    transformStamped_.transform.rotation.x = 0.0;
    transformStamped_.transform.rotation.y = 0.0;
    transformStamped_.transform.rotation.z = 0.0;
    transformStamped_.transform.rotation.w = 1.0;

    // Create a TransformBroadcaster
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Create a timer that calls the 'publishTransform' function every 50ms (20Hz)
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&LidarTFPublisher::publishTransform, this));
  }

private:
  void publishTransform()
  {
    // Update the timestamp and publish the transformation
    transformStamped_.header.stamp = now();
    broadcaster_->sendTransform(transformStamped_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTFPublisher>());
  rclcpp::shutdown();
  return 0;
}
