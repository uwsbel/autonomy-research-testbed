#include "yolov7/yolov7.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>

#include <opencv2/opencv.hpp>

#include <exception>
#include <functional>
#include <string>
#include <chrono>

YOLOv7::YOLOv7(const rclcpp::NodeOptions& options) : rclcpp::Node("yolov7", options) {

    // this->state = "";
    // this->prediction = {};
    // this->prediction["boxes"] = torch::tensor({});
    // this->prediction["labels"] = torch::tensor({});
    // this->prediction["scores"] = torch::tensor({});

    // this->threshold = 0.001;
    // this->device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;




    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, qos_profile_.depth), qos_profile_);

    using namespace std::placeholders;
    using std::placeholders::_1;

    img_subscriber_ = this->create_subscription<ImageMsg>("~/input/image", 10, std::bind(&YOLOv7::image_callback, this, _1));
    state_subscriber_ = this->create_subscription<VehicleStateMsg>("~/input/state", 10, std::bind(&YOLOv7::state_callback, this, _1));
    object_publisher_ = this->create_publisher<ObjectArrayMsg>("~/output/objects", 10);

}

YOLOv7::~YOLOv7() {}

void YOLOv7::state_callback(VehicleStateMsg msg) {
    this->state_msg = msg;
}

void YOLOv7::image_callback(ImageMsg msg) {
    this->go = true;
    this->image_msg = msg;
    
    int h = image_msg.height;
    int w = image_msg.width;

    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(this->image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;
    img.convertTo(img, CV_32F);
    img /= 255.0;


    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    img = img.rowRange(0, 704).colRange(0, img.cols).clone();

    // cv::cuda::GpuMat device_input;
    // device_input.upload(img);
    // device_input = device_input.permute(cv::cuda::CnPermute(2, 0, 1));
    // device_input = device_input.unsqueeze(0);

    // device_input = device_input.contiguous();

    // this->trt_context->executeV2(device_input.data, this->device_output.data);

    // // std::vector<float> boxes, scores, class_id;
    // this->nms(this->device_output[0]);
}

cv::Point3f YOLOv7::direction_to_pixel(const cv::Point2f& px) {
    float pt_x = -((px.x + 0.5) / this->camera_width * 2 - 1);
    float pt_y = -((px.y + 0.5) / this->camera_height * 2 - 1);
    pt_y *= this->camera_height / this->camera_width;

    float h_factor = this->camera_fov / (M_PI * 2.0);

    cv::Point3f direction(1, pt_x * h_factor, pt_y * h_factor);

    direction /= cv::norm(direction);

    return direction;
}

cv::Point3f YOLOv7::calculate_position_from_box(const cv::Rect& rectangle) {
    cv::Point2f top_px(0.5 * (rectangle.x + rectangle.width), rectangle.y);
    cv::Point2f bottom_px(0.5 * (rectangle.x + rectangle.width), rectangle.y + rectangle.height);

    cv::Point3f r1 = direction_to_pixel(top_px);
    cv::Point3f r2 = direction_to_pixel(bottom_px);

    float h = 0.078;

    float l2 = h / (r1.z * r2.x / r1.x - r2.z);
    cv::Point3f p2 = r2 * l2;

    cv::Point3f mount_pos(this->camera_position[0], this->camera_position[1], this->camera_position[2]);
    cv::Mat mount_rot(3, 3, CV_32F, this->orientation); // Use the constructor to initialize the matrix with orientation

    cv::Mat p2_mat = (cv::Mat_<float>(3, 1) << p2.x, p2.y, p2.z);
    cv::Mat result = mount_rot * p2_mat + (cv::Mat_<float>(3, 1) << mount_pos.x, mount_pos.y, mount_pos.z);

    return cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0));
}


void YOLOv7::pub_callback() {


    if (!this->go) {
        return;
    }

    ObjectArrayMsg ObjectArray;

    // for (int b = 0; b < this->boxes.size(); ++b)
    // {
    //     cv::Rect box = this->boxes[b];
    //     cv::Point3f position = this->calculate_position_from_box(box);

    //     ObjectMsg obj;
    //     obj.pose.position.x = position.x;
    //     obj.pose.position.y = position.y;
    //     obj.pose.position.z = position.z;
    //     obj.classification.classification = static_cast<int>(this->labels[b]) + 1;
    //     ObjectArray.objects.push_back(obj);

    //     // if (this->vis)
    //     // {
    //     //     std::string color = (this->labels[b] == 0) ? "r" : "g";
    //     //     float x = box.x; //- box.width / 2;
    //     //     float y = box.y; //- box.height / 2;
    //     //     float w = box.width - box.x;
    //     //     float h = box.height - box.y;
    //     //     cv::Rect2f rect(x, y, w, h);
    //     //     cv::Scalar rect_color(color == "r" ? 1 : 0, color == "g" ? 1 : 0, 0);
    //     //     cv::rectangle(this->im_show, rect, rect_color, 1);
    //     // }
    // }

    // // if (this->vis)
    // // {
    // //     cv::imshow("Image", this->im_show);
    // //     cv::waitKey(1);
    // //     this->counter++;
    // // }

    // ObjectArray.header.stamp = rclcpp::Time::now().to_msg();
    // this->pub_objects->publish(ObjectArray);
}