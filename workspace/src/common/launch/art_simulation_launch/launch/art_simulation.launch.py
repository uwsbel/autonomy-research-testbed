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

# external imports
from pathlib import Path

# ros imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument 


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(ld, "art_simulation/input/vehicle_inputs", "/control/vehicle_inputs")
    AddLaunchArgument(ld, "art_simulation/output/time", "/clock")
    AddLaunchArgument(ld, "art_simulation/output/vehicle", "/vehicle/state")
    AddLaunchArgument(ld, "art_simulation/output/camera", "/sensing/front_facing_camera/raw")
    AddLaunchArgument(ld, "ip", "")
    AddLaunchArgument(ld, "hostname", "")

    # TODO: Make a better exit case

    # -----
    # Nodes
    # -----

    node = Node(
        package='chrono_ros_bridge',
        executable='chrono_ros_bridge_node',
        name='chrono_ros_bridge',
        remappings=[
            ("~/input/driver_inputs", GetLaunchArgument("art_simulation/input/vehicle_inputs")),
            ("~/output/time", GetLaunchArgument("art_simulation/output/time")),
            ("~/output/vehicle", GetLaunchArgument("art_simulation/output/vehicle")),
            ("~/output/camera/front_facing_camera", GetLaunchArgument("art_simulation/output/camera")),
        ],
        parameters=[
             {"ip": GetLaunchArgument("ip")},
             {"hostname": GetLaunchArgument("hostname")},
             {"use_sim_time": GetLaunchArgument("use_sim_time")}
        ],
        condition=IfCondition(GetLaunchArgument("use_sim"))
    )
    ld.add_action(node)

    return ld

