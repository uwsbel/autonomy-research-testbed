#include "cone_detector/cone_detector.h"

#include <rclcpp_components/register_node_macro.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "art_perception_msgs/msg/ObjectArray.hpp"
#include "art_perception_msgs/msg/Object.hpp"


#define _USE_MATH_DEFINES
#include <cmath>


#include <string>
#include <filesystem>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <opencv2/opencv.hpp>
using namespace nvinfer1;



// using namespace std;




ConeDetector::ConeDetector(const rclcpp::NodeOptions& options) : rclcpp::Node("ConeDetector", options) {

    // this->state = "";
    // this->prediction = {};
    // this->prediction["boxes"] = torch::tensor({});
    // this->prediction["labels"] = torch::tensor({});
    // this->prediction["scores"] = torch::tensor({});

    // this->threshold = 0.001;
    // this->device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

    // if (!torch::cuda::is_available())
    // {
    //     RCLCPP_INFO(this->get_logger(), "CUDA support in Torch is not available. Either reconfigure with CUDA or fall back to a different perception node.");
    //     exit(1);
    // }

    // auto package_share_directory = ament_index_cpp::get_package_share_directory("cone_detector");
    
    // // load YOLO model
    // std::string model;
    // declare_parameter("model", model);
    // rclcpp::Parameter model_file = get_parameter("model");
    // if (model_file.get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET)
    //     model = model_file.as_string();
    // else
    //     RCLCPP_ERROR(this->get_logger(), "YOLO weights file path not set!");
    
    // // visualization
    // declare_parameter("vis", false);
    // this->vis = get_parameter("vis").get_parameter_value().get<bool>() ? "True" : "False";
    // RCLCPP_INFO(this->get_logger(), "vis: %s", this->vis);

    // // camera_calibration file
    // std::string camera_calibration;
    // declare_parameter("camera_calibration", camera_calibration);
    // camera_calibration = get_parameter("camera_calibration").as_string();

    // // load camera calibration -- hardcoding for now
    // // std::string camera_calibration_file;
    // // declare_parameter("camera_calibration_file", camera_calibration_file);
    // // camera_calibration_file = get_parameter("camera_calibration_file").as_string();

    // this->camera_width = 1280;
    // this->camera_height = 720;
    // this->camera_fov = 1.396;
    // this->camera_position = {0.0316,0,0.1933};
    // this->orientation = {0.9962, 0, 0.0872, 0,1,0, -0.0872,0,0.9962};

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_.history, qos_profile_.depth), qos_profile_);

    // subscribers and publishers
    using std::placeholders::_1;
    img_subscriber_ = this->create_subscription<ImageMsg>("~/input/image", qos, std::bind(&ConeDetector::image_callback, this, _1));
    state_subscriber_ = this->create_subscription<stateMsg>("~/input/vehicle_state", qos, std::bind(&ConeDetector::state_callback, this, _1));
    pub_objects = this->create_publisher<objectsMsg>("objects", 10);

    // initialize the TensorRT engine

    

    // std::filesystem::path packageShareDirectory = ament_index_cpp::get_package_share_directory("cone_detector");
    // std::filesystem::path model_file = model;
    // std::filesystem::path onnx_file = packageShareDirectory / model_file;
    // std::string engine_file = onnx_file + ".engine";

    // // to check if file exists
    // struct stat sb;


    // if (stat(engine_file, &sb) == 0) {
    //     IRuntime* runtime = createInferRuntime(gLogger);
    //     std::ifstream engineStream(engine_file, std::ios::binary);
    //     engine = runtime->deserializeCudaEngine(engineStream);

    //     if (engine == nullptr)
    //     {
    //         throw std::runtime_error("Engine is not initialized.");
    //     }
    // }
    // else {
    //     if (!std::ifstream(onnxFile))
    //     {
    //         throw std::runtime_error("Onnx file not found: " + onnxFile);
    //     }

    //     int explicitBatch = 1 << static_cast<int>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    //     IBuilder* builder = createInferBuilder(gLogger);
    //     INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    //     IParser* parser = createParser(*network, *logger);
    //     IBuilderConfig* config = builder->createBuilderConfig();
    //     config->setFlag(BuilderFlag::kGPU_FALLBACK);
    //     config->setMaxWorkspaceSize(1 << 30);

    //     std::ifstream onnxStream(onnxFile, std::ios::binary);
    //     if (!parser->parse(onnxStream))
    //     {
    //         std::string errorStr;
    //         for (int error = 0; error < parser->getNbErrors(); error++)
    //         {
    //             errorStr += parser->getError(error)->desc();
    //         }
    //         throw std::runtime_error(errorStr);
    //     }

    //     engine = builder->buildEngineWithConfig(*network, *config);

    //     if (engine == nullptr)
    //     {
    //         throw std::runtime_error("Engine is not initialized.");
    //     }

    //     std::ofstream engineStream(engine_file, std::ios::binary);
    //     engineStream.write(static_cast<char*>(engine->serialize()), engine->getSerializedSize());
    // }

    // IExecutionContext* context = engine->createExecutionContext();

    // int numBindings = engine->getNbBindings();
    // std::vector<int> inputShape, outputShape;
    // for (int i = 0; i < numBindings; i++) {
    //     if (engine->bindingIsInput(i)) {
    //         Dims inputDims = engine->getBindingDimensions(i);
    //         for (int d = 0; d < inputDims.nbDims; d++) {
    //             inputShape.push_back(inputDims.d[d]);
    //         }
    //         size_t inputSize = volume(inputDims) * engine->getMaxBatchSize() * sizeof(float16);
    //         // cudaMalloc(&buffers[i], inputSize);
    //     }
    //     else {
    //         Dims outputDims = engine->getBindingDimensions(i);
    //         for (int d = 0; d < outputDims.nbDims; d++) {
    //             outputShape.push_back(outputDims.d[d]);
    //         }
    //         // cudaMalloc(&buffers[i], outputSize);
    //     }
    // }

    // float confidenceThresh = 0.5;
    // float iouThresh = 0.45;
    // int maxDetections = 100;

    // add memory allocation for device output -- not sure how to do this

    // not sure how to do matplotlib with c++
}

ConeDetector::~ConeDetector() {}

// function to process data this node subscribes to
// std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> ConeDetector::nms(torch::Tensor pred) {
//     // Multiply class predictions by scores
//     pred.slice(1, 5) *= pred.slice(1, 4, 5);

//     // Change anchor point to corner rather than center
//     torch::Tensor boxes = pred.slice(1, 0, 4);
//     boxes.slice(1, 0, 1) -= boxes.slice(1, 2, 3) / 2;
//     boxes.slice(1, 1, 2) -= boxes.slice(1, 3, 4) / 2;
//     boxes.slice(1, 2, 3) += boxes.slice(1, 0, 1);
//     boxes.slice(1, 3, 4) += boxes.slice(1, 1, 2);

//     // Get the confidence and class ID
//     torch::Tensor scores, class_id;
//     std::tie(scores, class_id) = pred.slice(1, 5).max(1, false);

//     torch::Tensor mask = scores > this->confidence_thresh;
//     boxes = boxes.masked_select(mask.unsqueeze(1).expand_as(boxes)).reshape({ -1, 4 });
//     class_id = class_id.masked_select(mask);
//     scores = scores.masked_select(mask);

//     // Apply torchvision's batched_nms
//     torch::Tensor res = torch::ops::batched_nms(boxes, scores, class_id, this->iou_thresh);

//     if (res.size(0) > this->max_detections) {
//         res = res.slice(0, 0, this->max_detections);
//     }

//     torch::Tensor selected_boxes = boxes.index_select(0, res);
//     torch::Tensor selected_scores = scores.index_select(0, res);
//     torch::Tensor selected_class_id = class_id.index_select(0, res);

//     return std::make_tuple(selected_boxes, selected_scores, selected_class_id);
// }

void ConeDetector::state_callback(const VehicleStateMsgConstPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "state callback");

    // this->state_msg = msg;
}

void ConeDetector::image_callback(const ImageMsgConstPtr& msg) {
    RCLCPP_INFO(this->get_logger(), "image callback");


    // this->go = true;
    // this->image_msg = *msg;
    
    // int h = image_msg.height;
    // int w = image_msg.width;

    // cv_bridge::CvImagePtr cv_ptr;

    // try
    // {
    //     cv_ptr = cv_bridge::toCvCopy(this->image_msg, sensor_msgs::image_encodings::BGR8);
    // }
    // catch (cv_bridge::Exception& e)
    // {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // cv::Mat img = cv_ptr->image;
    // img.convertTo(img, CV_32F);
    // img /= 255.0;

    // cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    // img = img.rowRange(0, 704).colRange(0, img.cols).clone();

    // cv::cuda::GpuMat device_input;
    // device_input.upload(img);
    // device_input = device_input.permute(cv::cuda::CnPermute(2, 0, 1));
    // device_input = device_input.unsqueeze(0);

    // device_input = device_input.contiguous();

    // this->trt_context->executeV2(device_input.data, this->device_output.data);

    // // std::vector<float> boxes, scores, class_id;
    // this->nms(this->device_output[0]);
}

// cv::Point3f ConeDetector::direction_to_pixel(const cv::Point2f& px) {
//     float pt_x = -((px[0]+.5) / this->camera_width * 2 - 1);
//     float pt_y = -((px[1]+.5) / this->camera_height * 2 - 1);
//     pt_y *= this->camera_height / this->camera_width;

//     float h_factor = this->camera_fov / M_PI * 2.0;

//     cv::Point3f direction(1, pt_x * h_factor, pt_y * h_factor);

//     direction /= cv::norm(direction);

//     return direction
// }

// cv::Point3f ConeDetector::calculate_position_from_box(const cv::Rect& rectangle) {
//     cv::Point2f top_px(0.5 * (rectangle.x + rectangle.width), rectangle.y);
//     cv::Point2f bottom_px(0.5 * (rectangle.x + rectangle.width), rectangle.y + rectangle.height);

//     cv::Point3f r1 = directionToPixel(top_px);
//     cv::Point3f r2 = directionToPixel(bottom_px);

//     float h = 0.078;

//     float l2 = h / (r1.z * r2.x / r1.x - r2.z);
//     cv::Point3f p2 = r2 * l2;

//     cv::Point3f mount_pos(this->camera_params["position"][0], this->camera_params["position"][1], this->camera_params["position"][2]);
//     cv::Mat mount_rot(3, 3, CV_32F);
//     cv::Mat(this->camera_params["orientation"]).reshape(0, 3).copyTo(mount_rot);

//     cv::Mat p2_mat = (cv::Mat_<float>(3, 1) << p2.x, p2.y, p2.z);
//     cv::Mat result = mount_rot * p2_mat + (cv::Mat_<float>(3, 1) << mount_pos.x, mount_pos.y, mount_pos.z);

//     return cv::Point3f(result.at<float>(0, 0), result.at<float>(1, 0), result.at<float>(2, 0));
// }

void ConeDetector::pub_callback() {

    RCLCPP_INFO(this->get_logger(), "pub callback");

    // if (!this->go) {
    //     return;
    // }

    // ObjectArrayMsg ObjectArray;

    // for (int b = 0; b < this->boxes.size(); ++b)
    // {
    //     cv::Rect box = this->boxes[b];
    //     cv::Point3f position = this->calculatePositionFromBox(box);

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





