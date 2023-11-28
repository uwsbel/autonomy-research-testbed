#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#
import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleState, VehicleInput
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile

import numpy as np
import serial
import serial.tools.list_ports
import struct
import time


class SteeringServoDriver:
    def __init__(self):
        # === CONSTANTS ===
        # Pulse width as measured from the RC car receiver in milliseconds
        self.TRIM = -50
        self.PW_LEFT = 1700 + self.TRIM  # absolute left: 1980
        self.PW_RIGHT = 1300 + self.TRIM  # absolute right: 1000
        self.PW_CENTER = 1500 + self.TRIM

        # clamp response to achieve target
        self.current_pw = self.PW_CENTER
        self.current_steering = 0.0
        self.target_steering = 0.0
        self.MAX_STEERING_STEP = 0.1  # TODO find good value

    def step(self):
        # get steering value (-1,1) with a clamped step
        self.current_steering = np.clip(
            self.target_steering,
            self.current_steering - self.MAX_STEERING_STEP,
            self.current_steering + self.MAX_STEERING_STEP,
        )
        self.current_steering = np.clip(self.current_steering, -1.0, 1.0)

        # convert steering to duty cycle
        self.current_pw = self.PW_RIGHT + (self.PW_LEFT - self.PW_RIGHT) * (
            0.5 * self.current_steering + 0.5
        )

        self.current_pw = np.clip(
            self.current_pw,
            min(self.PW_RIGHT, self.PW_LEFT),
            max(self.PW_RIGHT, self.PW_LEFT),
        )

        return self.current_pw

    def setTargetSteering(self, steering):
        self.target_steering = np.clip(steering, -1.0, 1.0)


class MotorDriver:
    def __init__(self):
        # === CONSTANTS ===
        # Pulse width as measured from the RC car receiver in milliseconds
        self.TRIM = 0
        self.PW_BRAKE = 1600 + self.TRIM  # absolute full brake = 1980
        self.PW_FULL_THROTTLE = 1400 + self.TRIM  # absolute full throttle = 1000
        self.PW_NEUTRAL = 1500 + self.TRIM

        # clamp response to achieve target
        self.current_pw = self.PW_NEUTRAL
        self.current_throttle = 0.0
        self.target_throttle = 0.0
        self.MAX_THROTTLE_STEP = 0.1  # TODO find good value

        self.forward = True  # whether vehicle is in forward or reverse mode

    def Reverse(self):
        pass  # TODO

    def Forward(self):
        pass  # TODO

    def step(self):
        if self.forward:
            # get thottle and brake values (-1,1) with a clamped step
            self.current_throttle = np.clip(
                self.target_throttle,
                self.current_throttle - self.MAX_THROTTLE_STEP,
                self.current_throttle + self.MAX_THROTTLE_STEP,
            )
            self.current_throttle = np.clip(self.current_throttle, -1.0, 1.0)
            # convert steering to duty cycle
            if self.current_throttle >= 0:
                self.current_pw = self.PW_FULL_THROTTLE + (
                    self.PW_NEUTRAL - self.PW_FULL_THROTTLE
                ) * (1 - self.current_throttle)
            else:
                self.current_pw = (
                    self.PW_NEUTRAL
                    + (self.PW_BRAKE - self.PW_NEUTRAL) * -self.current_throttle
                )

            self.current_pw = np.clip(
                self.current_pw,
                min(self.PW_BRAKE, self.PW_FULL_THROTTLE),
                max(self.PW_BRAKE, self.PW_FULL_THROTTLE),
            )
            return self.current_pw

        else:
            pass

    def setTargetThrottle(self, throttle, braking):
        self.target_throttle = np.clip(throttle - braking, -1.0, 1.0)
        return self.target_throttle


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver")

        # update frequencies of this node
        self.freq = 20.0  # PWM is at 60Hz, so we should not overwrite previous signal too quickly

        # data that will be used by this class
        self.vehicle_cmd = VehicleInput()
        self.stale_timer = (
            0  # will kill motors if no commands for specific amount of time
        )
        self.KILL_TIME = (
            0.3  # time after which motors will be killed if no new commands given
        )

        # subscriber
        qos_profile = QoSProfile(depth=1)
        qos_profile.history = QoSHistoryPolicy.KEEP_LAST
        self.sub_vehicle_cmd = self.create_subscription(
            VehicleInput, "~/output/vehicle_inputs", self.control_callback, qos_profile
        )

        # call the driver callback even if we haven't heard from the subscribers
        self.timer = self.create_timer(1 / self.freq, self.update_motors)

        # motor and servo objects
        self.motor = MotorDriver()
        self.servo = SteeringServoDriver()

        BAUD_RATE = 250000
        PORT = "/dev/ttyUSB0"
        TIMEOUT = 0.1
        self.arduino = serial.Serial(port=PORT, baudrate=BAUD_RATE, timeout=TIMEOUT)

    # function to process data this class subscribes to
    def control_callback(self, msg):
        self.vehicle_cmd = msg  # save the message
        # self.get_logger().info("vehicle_cmd msg='%s'" % self.vehicle_cmd)
        self.stale_timer = 0  # reset watchdog timer

    def update_motors(self):
        # print("Sending commands to motor and steering servo")
        self.stale_timer += 1 / self.freq
        if self.stale_timer >= self.KILL_TIME:
            self.vehicle_cmd.throttle = 0.0
            self.vehicle_cmd.braking = 0.0

        # self.get_logger().info("Motors '%s'" % self.vehicle_cmd)

        self.servo.setTargetSteering(self.vehicle_cmd.steering)
        target = self.motor.setTargetThrottle(
            self.vehicle_cmd.throttle, self.vehicle_cmd.braking
        )
        # self.get_logger().info("target throttle='%s (%s - %s)'" % (target,self.vehicle_cmd.throttle,self.vehicle_cmd.braking))
        servo_pw = int(self.servo.step())
        esc_pw = int(self.motor.step())

        msg_size = struct.pack("<B", 4)
        msg_data = struct.pack("<2H", *[servo_pw, esc_pw])
        msg = b"".join([msg_size, msg_data])
        t0 = time.time()
        self.arduino.write(msg)
        t1 = time.time()
        # self.get_logger().info("servo=%s,esc=%s, serial time=%s" % (servo_pw,esc_pw,str(t1-t0)))


def main(args=None):
    # print("=== Starting MotorDriverNode ===")
    rclpy.init(args=args)
    driver = MotorDriverNode()
    rclpy.spin(driver)

    driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
