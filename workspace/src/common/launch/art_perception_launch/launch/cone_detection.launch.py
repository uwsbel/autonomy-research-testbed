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

    AddLaunchArgument("input/image", "/sensing/fwc/raw/image")
    AddLaunchArgument("input/vehicle_state", "/vehicle/state")
    AddLaunchArgument("output/objects", "/perception/objects")

    AddLaunchArgument("use_sim_time", "False")
    AddLaunchArgument("model", "data/model_refined")
    AddLaunchArgument("camera_calibration_file", "data/calibration.json")
    AddLaunchArgument("vis", "False")

    # -----
    # Nodes
    # -----

    node = Node(
        package='cone_detector',
        namespace='',
        executable='object_recognition',
        name='object_recognition',
        remappings=[
            ("~/input/image", LaunchConfiguration("input/image")),
            ("~/input/vehicle_state", LaunchConfiguration("input/vehicle_state")),
            ("~/output/objects", LaunchConfiguration("output/objects"))
        ],
        parameters=[
             {"use_sim_time": LaunchConfiguration("use_sim_time")},
             {"model":LaunchConfiguration("model")},
             {"camera_calibration_file":LaunchConfiguration("camera_calibration_file",)},
             {"vis": LaunchConfiguration("vis")}
        ]
    )
    launch_description.add_action(node)

    return launch_description
