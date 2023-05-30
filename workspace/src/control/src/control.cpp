/******************************************************************************
 * BSD 3-Clause License
 * 
 * Copyright (c) 2022 University of Wisconsin - Madison
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/path.hpp"
#include "art_msgs/msg/vehicle_input.hpp"
#include "art_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control.h"

// /**
//  * @brief Control Node class
//  * 
//  * Publishes steering, throttle & breaking values
//  * Subscribes to Navigation Path & ART/Chrono Vehicle State messages
//  */
// class controlNode : public rclcpp::Node {
//   public:
//     explicit controlNode() : Node("control") {
//         this->startTime = this->get_clock()->now().nanoseconds() / 1e9;

//         /*
//             Update frequency of this node
//         */
//         this->nodeUpdateFrequency = 10.0;

//         /*
//             Default settings
//         */
//         this->mode = "PID";
//         this->file = "";
//         this->steering = 0.0;
//         this->throttle = 0.0;
//         this->braking = 0.0;

//         std::string package_share_directory = ament_index_cpp::get_package_share_directory("control");

//         /*
//             Declaring default parameters and fetching ROS launch parameters
//             For default launch parameters, refer workspace/src/common/launch/art_<node>_launch/ of the node
//         */
//         this->declare_parameter("control_mode", "PID");
//         this->declare_parameter("control_file", "");
//         this->declare_parameter("steering_gain", 1.0);
//         this->declare_parameter("throttle_gain", 1.0);
//         this->declare_parameter("use_sim_msg", "False");

//         this->mode = this->get_parameter("control_mode").as_string();
//         this->file = this->get_parameter("control_file").as_string();
//         this->steeringGain = this->get_parameter("steering_gain").as_double();
//         this->throttleGain = this->get_parameter("throttle_gain").as_double();
//         this->useSimMsg = this->get_parameter("use_sim_msg").as_string();

//         if (this->file == "") {
//             this->mode = "PID";
//         } else {
//             std::string filePath = package_share_directory + '/' + this->file;

//             std::ifstream inputFile(filePath);

//             /*
//                 Parsing input .csv file
//             */
//             if (inputFile) {
//                 std::string line;

//                 while (std::getline(inputFile, line)) {
//                     std::stringstream fileStream(line);
//                     std::string element;

//                     while (std::getline(fileStream, element, ',')) {
//                         this->recordedInputs.push_back(std::stof(element));
//                     }
//                 }
//                 inputFile.close();
//             }
//         }

//         this->path = nav_msgs::msg::Path();

//         /*
//             Waits for first path if using PID, otherwise runs right away
//         */
//         if (this->mode == "File")
//             this->go = true;
//         else
//             this->go = false;

//         rclcpp::QoS qos_profile(1);
//         qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

//         /*
//             Nav path & ART/Chrono Vehicle State subscribers
//         */
//         this->pathSubscriber_ = this->create_subscription<nav_msgs::msg::Path>(
//             "~/input/path", qos_profile, std::bind(&controlNode::pathCallback, this, std::placeholders::_1));
//         this->stateSubscriber_ = this->create_subscription<art_msgs::msg::VehicleState>(
//             "~/input/vehicle_state", qos_profile, std::bind(&controlNode::stateCallback, this, std::placeholders::_1));

//         /*
//             Publisher for this node
//         */
//         this->vehicleCmdPublisher_ = this->create_publisher<art_msgs::msg::VehicleInput>("~/output/vehicle_inputs", 10);
//         this->timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / this->nodeUpdateFrequency),
//                                                std::bind(&controlNode::pubCallback, this));
//     }

//   private:
//     uint nodeUpdateFrequency;
//     double steering;
//     double throttle;
//     double braking;
//     bool go;

//     /*******************************
//             LAUNCH PARAMETERS
//     *******************************/
//     double steeringGain;
//     double throttleGain;
//     std::string mode;
//     std::string file;
//     std::string useSimMsg;
//     std::vector<float> recordedInputs;

//     /*******************************
//             MESSAGE CLASSES
//     *******************************/
//     nav_msgs::msg::Path path;
//     art_msgs::msg::VehicleState state;

//     /*******************************
//             TIMING VARIABLES
//     *******************************/
//     double startTime;
//     rclcpp::TimerBase::SharedPtr timer_;

//     /******************************
//         PUBLISHERS & SUBSCRIBERS
//     *******************************/
//     rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscriber_;
//     rclcpp::Subscription<art_msgs::msg::VehicleState>::SharedPtr stateSubscriber_;
//     rclcpp::Publisher<art_msgs::msg::VehicleInput>::SharedPtr vehicleCmdPublisher_;

//     /******************************
//             MEMBER FUNCTIONS
//     ******************************/
//     /**
//      * @brief Callback function for pathSubscriber_
//      * 
//      * This function takes the Path message from nav_msgs, stores in a class variable
//      * and sets to vehicle to motion
//      * 
//      * @param msg Input Path message
//      */
//     void pathCallback(const nav_msgs::msg::Path msg) {
//         go = true;
//         path = msg;
//     }

//     /**
//      * @brief Callback function for stateSubscriber_
//      * 
//      * This function takes a State message from ART or Chrono & stores in a class variable
//      * 
//      * @param msg Input Vehicle State message
//      */
//     void stateCallback(const art_msgs::msg::VehicleState msg) {
//         state = msg;
//     }

//     /**
//      * @brief The publisher callback function for this node
//      * Contructs the message to be sent and publishes.
//      * 
//      */
//     void pubCallback(void) {
//         if (!go) {
//             return;
//         }

//         /*
//             If input from .csv, calculate steering from file
//         */
//         if (mode == "File") {
//             calculateFromFile();
//         } else if (mode == "PID" && path.poses.size() > 0) {
//             steering = steeringGain * (path.poses[0].pose.position.y / path.poses[0].pose.position.x);
//         }

//         /*
//             Only doing lateral conmtrol for now
//         */
//         throttle = throttleGain * 0.55;

//         art_msgs::msg::VehicleInput msg = art_msgs::msg::VehicleInput();

//         msg.steering = clip(steering, -1, 1);
//         msg.throttle = clip(throttle, 0, 1);
//         msg.braking = clip(braking, 0, 1);

//         /*
//             TODO: Remove if condition after adding "header" to choro_msgs
//         */
//         if(useSimMsg == "True") {
//             msg.header.stamp = this->get_clock()->now();
//         }

//         vehicleCmdPublisher_->publish(msg);
//     }

//     /**
//      * @brief Calculates the throttle, braking and steering values from file
//      * 
//      */
//     void calculateFromFile(void) {
//         long int timeDiff = (get_clock()->now().nanoseconds() / 1e9) - startTime;

//         throttle = interpolate(timeDiff, &recordedInputs, 1);
//         braking = interpolate(timeDiff, &recordedInputs, 2);
//         steering = interpolate(timeDiff, &recordedInputs, 3);
//     }

//     /**
//      * @brief A function to clip values to be between max & min
//      * 
//      * An equivalent to the numpy clip function
//      * 
//      * if input > max, return max,
//      * else if input < min, return min,
//      * else return input
//      * 
//      * @param input The input value to clip
//      * @param min Minimum limit
//      * @param max Maximum limit
//      * @return double Clipped value
//      */
//     double clip(double input, double min, double max) {
//         return std::max(min, std::min(max, input));
//     }

//     /**
//      * @brief Interpolate input with respect to the recorded inputs
//      * 
//      * Equivalent to numpy interp function.
//      * Using Lagrange's interpolation method
//      * 
//      * @param input The data point to interpolate
//      * @param recordedInputs Array of data points recorded
//      * @param type Choose between throttle, braking and steering
//      * (Relates to the order of data in the recorded data points)
//      * 1 - Throttle, 2 - Braking, 3 - Steering
//      * @return double Interpolation result 
//      */
//     double interpolate(long int input, const std::vector<float>* recordedInputs, uint8_t type) {
//         long int n = recordedInputs->size();
//         double result = 0;
//         for (long int i = 0; i < n; i += 4) {
//             double temp = 1;
//             for (long int j = 0; j < n; j += 4) {
//                 if (i != j) {
//                     temp = temp * (input - recordedInputs->at(j)) / (recordedInputs->at(i) - recordedInputs->at(j));
//                 }
//             }
//             result = result + temp * recordedInputs->at(i + type);
//         }
//         return result;
//     }
// };

controlNode::controlNode() : rclcpp::Node("control") {
    this->startTime = this->get_clock()->now().nanoseconds() / 1e9;

    /*
        Update frequency of this node
    */
    this->nodeUpdateFrequency = 10.0;

    /*
        Default settings
    */
    this->mode = "PID";
    this->file = "";
    this->steering = 0.0;
    this->throttle = 0.0;
    this->braking = 0.0;

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("control");

    /*
        Declaring default parameters and fetching ROS launch parameters
        For default launch parameters, refer workspace/src/common/launch/art_<node>_launch/ of the node
    */
    this->declare_parameter("control_mode", "PID");
    this->declare_parameter("control_file", "");
    this->declare_parameter("steering_gain", 1.0);
    this->declare_parameter("throttle_gain", 1.0);
    this->declare_parameter("use_sim_msg", "False");

    this->mode = this->get_parameter("control_mode").as_string();
    this->file = this->get_parameter("control_file").as_string();
    this->steeringGain = this->get_parameter("steering_gain").as_double();
    this->throttleGain = this->get_parameter("throttle_gain").as_double();
    this->useSimMsg = this->get_parameter("use_sim_msg").as_string();

    if (this->file == "") {
        this->mode = "PID";
    } else {
        std::string filePath = package_share_directory + '/' + this->file;

        std::ifstream inputFile(filePath);

        /*
            Parsing input .csv file
        */
        if (inputFile) {
            std::string line;

            while (std::getline(inputFile, line)) {
                std::stringstream fileStream(line);
                std::string element;

                while (std::getline(fileStream, element, ',')) {
                    this->recordedInputs.push_back(std::stof(element));
                }
            }
            inputFile.close();
        }
    }

    this->path = nav_msgs::msg::Path();

    /*
        Waits for first path if using PID, otherwise runs right away
    */
    if (this->mode == "File")
        this->go = true;
    else
        this->go = false;

    rclcpp::QoS qos_profile(1);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

    /*
        Nav path & ART/Chrono Vehicle State subscribers
    */
    this->pathSubscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "~/input/path", qos_profile, std::bind(&controlNode::pathCallback, this, std::placeholders::_1));
    this->stateSubscriber_ = this->create_subscription<art_msgs::msg::VehicleState>(
        "~/input/vehicle_state", qos_profile, std::bind(&controlNode::stateCallback, this, std::placeholders::_1));

    /*
        Publisher for this node
    */
    this->vehicleCmdPublisher_ = this->create_publisher<art_msgs::msg::VehicleInput>("~/output/vehicle_inputs", 10);
    this->timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / this->nodeUpdateFrequency),
                                            std::bind(&controlNode::pubCallback, this));
}

void controlNode::pathCallback(const nav_msgs::msg::Path msg) {
    this->go = true;
    this->path = msg;
}

void controlNode::stateCallback(const art_msgs::msg::VehicleState msg) {
    this->state = msg;
}

void controlNode::pubCallback(void) {
    if (!this->go) {
        return;
    }

    /*
        If input from .csv, calculate steering from file
    */
    if (this->mode == "File") {
        calculateFromFile();
    } else if (this->mode == "PID" && path.poses.size() > 0) {
        this->steering = this->steeringGain * (path.poses[0].pose.position.y / path.poses[0].pose.position.x);
    }

    /*
        Only doing lateral conmtrol for now
    */
    this->throttle = this->throttleGain * 0.55;

    art_msgs::msg::VehicleInput msg = art_msgs::msg::VehicleInput();

    msg.steering = clip(this->steering, -1, 1);
    msg.throttle = clip(this->throttle, 0, 1);
    msg.braking = clip(this->braking, 0, 1);

    /*
        TODO: Remove if condition after adding "header" to choro_msgs
    */
    if(this->useSimMsg == "True") {
        msg.header.stamp = this->get_clock()->now();
    }

    vehicleCmdPublisher_->publish(msg);
}

void controlNode::calculateFromFile(void) {
    long int timeDiff = (get_clock()->now().nanoseconds() / 1e9) - startTime;

    this->throttle = interpolate(timeDiff, &recordedInputs, 1);
    this->braking = interpolate(timeDiff, &recordedInputs, 2);
    this->steering = interpolate(timeDiff, &recordedInputs, 3);
}

double controlNode::clip(double input, double min, double max) {
    return std::max(min, std::min(max, input));
}

double controlNode::interpolate(long int input, const std::vector<float>* recordedInputs, uint8_t type) {
    long int n = recordedInputs->size();
    double result = 0;
    for (long int i = 0; i < n; i += 4) {
        double temp = 1;
        for (long int j = 0; j < n; j += 4) {
            if (i != j) {
                temp = temp * (input - recordedInputs->at(j)) / (recordedInputs->at(i) - recordedInputs->at(j));
            }
        }
        result = result + temp * recordedInputs->at(i + type);
    }
    return result;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controlNode>());
    rclcpp::shutdown();

    return 0;
}
