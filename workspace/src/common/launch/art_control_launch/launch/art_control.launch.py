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

# ros imports
from launch import LaunchDescription
from launch_ros.actions import Node

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(ld, "art_control/input/path", "/path_planning/path")
    AddLaunchArgument(ld, "art_control/input/vehicle_state", "/vehicle/state")
    AddLaunchArgument(ld, "art_control/output/vehicle_inputs", "/control/vehicle_inputs")

    AddLaunchArgument(ld, "control_mode", "PID")
    AddLaunchArgument(ld, "control_file", "data/smallest_radius_right.csv")
    AddLaunchArgument(ld, "steering_gain", "1.6")
    AddLaunchArgument(ld, "throttle_gain", "0.08")
    AddLaunchArgument(ld, "use_sim_time", "False")

    # -----
    # Nodes
    # -----

    node = Node(
            package='control',
            executable='pid',
            name='pid',
            remappings=[
                ("~/input/path", GetLaunchArgument("art_control/input/path")),
                ("~/input/vehicle_state", GetLaunchArgument("art_control/input/vehicle_state")),
                ("~/output/vehicle_inputs", GetLaunchArgument("art_control/output/vehicle_inputs"))
            ],
            parameters=[
                {"input": GetLaunchArgument("art_control/input/path")},
                {"control_mode": GetLaunchArgument("control_mode")},
                {"control_file": GetLaunchArgument("control_file")},
                {"steering_gain": GetLaunchArgument("steering_gain")},
                {"throttle_gain": GetLaunchArgument("throttle_gain")},
                {"use_sim_time": GetLaunchArgument("use_sim_time")}
            ]
        )
    ld.add_action(node)

    return ld 
