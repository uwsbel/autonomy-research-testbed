#include "yolov7/yolov7.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<YOLOv7>(options);

    RCLCPP_INFO(node->get_logger(), "yolov7 node started up!");
    // actually run the node
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
    return 0;
}
