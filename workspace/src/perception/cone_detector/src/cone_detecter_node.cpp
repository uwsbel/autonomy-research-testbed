#include "cone_detector/cone_detector.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto node = std::make_shared<cone_detector>(options);

    RCLCPP_INFO(node->get_logger(), "cone_detector node started up!");
    // actually run the node
    rclcpp::spin(node);  // should not return
    rclcpp::shutdown();
    return 0;
}
