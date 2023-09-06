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

#ifndef CONTROL_H
#define CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "art_msgs/msg/vehicle_input.hpp"
#include "art_msgs/msg/vehicle_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

/**
 * @brief Control Node class
 *
 * Publishes steering, throttle & breaking values
 * Subscribes to Navigation Path & ART/Chrono Vehicle State messages
 */
class control : public rclcpp::Node {
    /*******************************
                PRIVATE
    *******************************/
  private:
    uint nodeUpdateFrequency;
    double steering;
    double throttle;
    double braking;
    bool go;

    /*******************************
            LAUNCH PARAMETERS
    *******************************/
    double steeringGain;
    double throttleGain;
    std::string mode;
    std::string file;
    std::string useSimMsg;
    std::vector<float> recordedInputs;

    /*******************************
            MESSAGE CLASSES
    *******************************/
    nav_msgs::msg::Path path;
    art_msgs::msg::VehicleState state;

    /*******************************
            TIMING VARIABLES
    *******************************/
    double startTime;
    rclcpp::TimerBase::SharedPtr timer_;

    /******************************
        PUBLISHERS & SUBSCRIBERS
    *******************************/
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscriber_;
    rclcpp::Subscription<art_msgs::msg::VehicleState>::SharedPtr stateSubscriber_;
    rclcpp::Publisher<art_msgs::msg::VehicleInput>::SharedPtr vehicleCmdPublisher_;

    /******************************
            MEMBER FUNCTIONS
    ******************************/
    /**
     * @brief Callback function for pathSubscriber_
     *
     * This function takes the Path message from nav_msgs, stores in a class variable
     * and sets to vehicle to motion
     *
     * @param msg Input Path message
     */
    void pathCallback(const nav_msgs::msg::Path);

    /**
     * @brief Callback function for stateSubscriber_
     *
     * This function takes a State message from ART or Chrono & stores in a class variable
     *
     * @param msg Input Vehicle State message
     */
    void stateCallback(const art_msgs::msg::VehicleState);

    /**
     * @brief The publisher callback function for this node
     * Contructs the message to be sent and publishes.
     *
     */
    void pubCallback(void);

    /**
     * @brief Calculates the throttle, braking and steering values from file
     *
     */
    void calculateFromFile(void);

    /**
     * @brief A function to clip values to be between max & min
     *
     * An equivalent to the numpy clip function
     *
     * if input > max, return max,
     * else if input < min, return min,
     * else return input
     *
     * @param input The input value to clip
     * @param min Minimum limit
     * @param max Maximum limit
     * @return double Clipped value
     */
    double clip(double, double, double);

    /**
     * @brief Interpolate input with respect to the recorded inputs
     *
     * Equivalent to numpy interp function.
     * Using Lagrange's interpolation method
     *
     * @param input The data point to interpolate
     * @param recordedInputs Array of data points recorded
     * @param type Choose between throttle, braking and steering
     * (Relates to the order of data in the recorded data points)
     * 1 - Throttle, 2 - Braking, 3 - Steering
     * @return double Interpolation result
     */
    double interpolate(long int, const std::vector<float>*, uint8_t);

    /*******************************
                PUBLIC
    *******************************/
  public:
    explicit control();
};

#endif
