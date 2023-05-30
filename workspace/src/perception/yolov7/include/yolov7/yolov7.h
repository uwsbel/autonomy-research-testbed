#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/synchronizer.h>

#include "art_msgs/msg/vehicle_state.hpp"
#include "art_perception_msgs/msg/object_array.hpp"
#include "art_perception_msgs/msg/object.hpp"
#include <opencv2/opencv.hpp>

// #include <NvInfer.h>



class YOLOv7 : public rclcpp::Node {
  public:
    explicit YOLOv7(const rclcpp::NodeOptions& options);
    ~YOLOv7();

    // --------
    // Typedefs
    // --------

    typedef sensor_msgs::msg::Image ImageMsg;
    typedef std::shared_ptr<ImageMsg> ImageMsgPtr;
    typedef std::shared_ptr<ImageMsg const> ImageMsgConstPtr;

    typedef art_msgs::msg::VehicleState VehicleStateMsg;
    typedef std::shared_ptr<art_msgs::msg::VehicleState> VehicleStateMsgPtr;
    typedef std::shared_ptr<art_msgs::msg::VehicleState const> VehicleStateMsgConstPtr;

    typedef art_perception_msgs::msg::ObjectArray ObjectArrayMsg;
    typedef art_perception_msgs::msg::Object ObjectMsg;


    // -------------
    // ROS Functions
    // -------------
    // void imageCallback(ImageMsg img);

    void image_callback(ImageMsg img);
    void state_callback(VehicleStateMsg state_msg);
    void pub_callback();


  private:

    // class Logger : public nvinfer1::ILogger
    // {
    // public:
    //     void log(Severity severity, const char* msg) override {
    //         // remove this 'if' if you need more logged info
    //         if ((severity == Severity::kERROR) || (severity == Severity::kINTERNAL_ERROR)) {
    //             std::cout << msg << "n";
    //         }
    //     }
    // } gLogger;
    // ----------------
    // Helper Functions
    // ----------------

    cv::Point3f direction_to_pixel(const cv::Point2f& px);
    cv::Point3f calculate_position_from_box(const cv::Rect& rectangle);

    // ---------------
    // Class Variables
    // ---------------

    // std::string = state;
    // std::map<std::string, torch::Tensor> prediction;
    // float threshold;

    // torch::Device device;

    
    float camera_width;
    float camera_height;
    float camera_fov;
    float camera_position[3];
    float orientation[9];


    VehicleStateMsg state_msg;
    ImageMsg image_msg;


    bool go = false;

    rmw_qos_profile_t qos_profile_ = rmw_qos_profile_sensor_data;

    rclcpp::Subscription<ImageMsg>::SharedPtr img_subscriber_;
    rclcpp::Subscription<VehicleStateMsg>::SharedPtr state_subscriber_;
    rclcpp::Publisher<ObjectArrayMsg>::SharedPtr object_publisher_;

};

// class Logger : public nvinfer1::ILogger
// {
// public:
//     void log(Severity severity, const char* msg) override {
//         // remove this 'if' if you need more logged info
//         if ((severity == Severity::kERROR) || (severity == Severity::kINTERNAL_ERROR)) {
//             std::cout << msg << "n";
//         }
//     }
// } gLogger;