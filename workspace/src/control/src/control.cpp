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

#include "control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

control::control() : rclcpp::Node("control") {
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
    this->go = (this->mode == "File");

    rclcpp::QoS qos_profile(1);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

    /*
        Nav path & ART/Chrono Vehicle State subscribers
    */
    this->pathSubscriber_ = this->create_subscription<nav_msgs::msg::Path>(
        "~/input/path", qos_profile, std::bind(&control::pathCallback, this, std::placeholders::_1));
    this->stateSubscriber_ = this->create_subscription<art_msgs::msg::VehicleState>(
        "~/input/vehicle_state", qos_profile, std::bind(&control::stateCallback, this, std::placeholders::_1));

    /*
        Publisher for this node
    */
    this->vehicleCmdPublisher_ = this->create_publisher<art_msgs::msg::VehicleInput>("~/output/vehicle_inputs", 10);
    this->timer_ = this->create_wall_timer(std::chrono::duration<double>(1 / this->nodeUpdateFrequency),
                                           std::bind(&control::pubCallback, this));
}

void control::pathCallback(const nav_msgs::msg::Path msg) {
    this->go = true;
    this->path = msg;
}

void control::stateCallback(const art_msgs::msg::VehicleState msg) {
    this->state = msg;
}

void control::pubCallback(void) {
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

    msg.header.stamp = this->get_clock()->now();

    vehicleCmdPublisher_->publish(msg);
}

void control::calculateFromFile(void) {
    long int timeDiff = (get_clock()->now().nanoseconds() / 1e9) - startTime;

    this->throttle = interpolate(timeDiff, &recordedInputs, 1);
    this->braking = interpolate(timeDiff, &recordedInputs, 2);
    this->steering = interpolate(timeDiff, &recordedInputs, 3);
}

double control::clip(double input, double min, double max) {
    return std::max(min, std::min(max, input));
}

double control::interpolate(long int input, const std::vector<float>* recordedInputs, uint8_t type) {
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
    rclcpp::spin(std::make_shared<control>());
    rclcpp::shutdown();

    return 0;
}
