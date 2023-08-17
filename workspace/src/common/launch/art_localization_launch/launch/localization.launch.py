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
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    launch_description = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    def AddLaunchArgument(arg, default):
        launch_description.add_action(
            DeclareLaunchArgument(
                arg,
                default_value=TextSubstitution(text=default)
            )
        )

    #AddLaunchArgument("input/gps","/fix")
    #AddLaunchArgument("input/mag", "/imu/mag")
    AddLaunchArgument("input/gps", "/chrono_ros_bridge/output/gps/data")
    AddLaunchArgument("input/mag", "/chrono_ros_bridge/output/magnetometer/data")

    AddLaunchArgument("input/groundTruth", "/vehicle/state")
    AddLaunchArgument("input/gyro", "/chrono_ros_bridge/output/gyroscope/data")
    AddLaunchArgument("input/accel", "/chrono_ros_bridge/output/accelerometer/data")
    AddLaunchArgument("input/vehicleInput", "/control/vehicle_inputs")
    AddLaunchArgument("output/vehicle_state", "/vehicle_state")

    AddLaunchArgument("vis", "False")
    AddLaunchArgument("use_sim_msg", "False")
    AddLaunchArgument("use_sim_time", "False")
    AddLaunchArgument("SE_mode", "GT")

    # -----
    # Nodes
    # -----

    node = Node(
        package='localization_py',
        namespace='',
        executable='state_estimation',
        name='state_estimation',
        remappings=[
                ("~/input/gps", LaunchConfiguration("input/gps")),
                ("~/input/groundTruth", LaunchConfiguration("input/groundTruth")),
                ("~/input/mag", LaunchConfiguration("input/mag")),
                ("~/input/gyro", LaunchConfiguration("input/gyro")),
                ("~/input/accel", LaunchConfiguration("input/accel")),
                ("~/output/vehicle_state", LaunchConfiguration("output/vehicle_state")),
                ("~/input/vehicleInput", LaunchConfiguration("input/vehicleInput")),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"use_sim_msg": LaunchConfiguration("use_sim_msg")},
            {"vis": LaunchConfiguration("vis")},
            {"SE_mode": LaunchConfiguration("SE_mode")}
        ]
    )
    launch_description.add_action(node)

    return launch_description