#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <message_filters/subscriber.h>
#include "art_msgs/msg/VehicleState.hpp"
#include "art_perception_msgs/msg/ObjectArray.hpp"
#include "art_perception_msgs/msg/Object.hpp"
#include <opencv2/opencv.hpp>



class ConeDetector : public rclcpp::Node {
    public:
        explicit ConeDetector(const rclcpp::NodeOptions& options);
        ~ConeDetector();

        // --------
        // Typedefs
        // --------

        typedef sensor_msgs::msg::Image ImageMsg;
        typedef std::shared_ptr<sensor_msgs::msg::Image> ImageMsgPtr;
        typedef std::shared_ptr<sensor_msgs::msg::Image const> ImageMsgConstPtr;

        typedef art_msgs::msg::VehicleState VehicleStateMsg;
        typedef std::shared_ptr<art_msgs::msg::VehicleState> VehicleStateMsgPtr;
        typedef std::shared_ptr<art_msgs::msg::VehicleState const> VehicleStateMsgConstPtr;

        typedef art_perception_msgs::msg::ObjectArray ObjectArrayMsg;
        typedef art_perceptoin_msgs::msg::Object ObjectMsg;




        // -------------
        // ROS Functions
        // -------------
        
        void state_callback(const VehicleStateMsgConstPtr& state_msg);
        void image_callback(const ImageMsgConstPtr& image_msg);
        void pub_callback();
    
    private:

        class Logger : public nvinfer1::ILogger
        {
        public:
            void log(Severity severity, const char* msg) override {
                // remove this 'if' if you need more logged info
                if ((severity == Severity::kERROR) || (severity == Severity::kINTERNAL_ERROR)) {
                    std::cout << msg << "n";
                }
            }
        } gLogger;

        // -------------
        // Helper Functions
        // -------------

        
        // std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> nms(torch::Tensor pred)
        // cv::Point3f direction_to_pixel(const cv::Point2f& px);
        // cv::Point3f calculate_position_from_box(const cv::Rect& rectangle);

        // ---------------
        // Class Variables
        // ---------------

        float camera_width;
        float camera_height;
        float camera_fov;
        float[] camera_position;
        float[] orientation;

        rmw_qos_profile_t qos_profile_ = rmw_qos_profile_sensor_data;

        VehicleStateMsgConstPtr state_msg;
        ImageMsgConstPtr image_msg;
        

        bool go = false;



        rclcpp::Subscription<ImageMsg>::SharedPtr img_subscriber_;

};



