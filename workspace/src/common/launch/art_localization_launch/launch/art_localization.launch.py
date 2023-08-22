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

    AddLaunchArgument(ld, "art_localization/input/gps", "/sensing/gps/data")
    AddLaunchArgument(ld, "art_localization/input/magnetometer", "/sensing/magnetometer/data")
    AddLaunchArgument(ld, "art_localization/input/gyroscope", "/sensing/gyroscope/data")
    AddLaunchArgument(ld, "art_localization/input/accelerometer", "/sensing/accelerometer/data")
    AddLaunchArgument(ld, "art_localization/output/vehicle_state", "/vehicle/filtered_state")

    AddLaunchArgument(ld, "vis", "False")

    # -----
    # Nodes
    # -----

    node = Node(
        package='localization_py',
        executable='state_estimation',
        name='state_estimation',
        remappings=[
                ("~/input/gps", GetLaunchArgument("art_localization/input/gps")),
                ("~/input/magnetometer", GetLaunchArgument("art_localization/input/magnetometer")),
                ("~/input/gyroscope", GetLaunchArgument("art_localization/input/gyroscope")),
                ("~/input/accelerometer", GetLaunchArgument("art_localization/input/accelerometer")),
                ("~/output/vehicle_state", GetLaunchArgument("art_localization/output/vehicle_state")),
        ],
        parameters=[
            {"vis": GetLaunchArgument("vis")},
            {"use_sim_time": GetLaunchArgument("use_sim_time")}
        ]
    )
    ld.add_action(node)

    return ld 
