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

# Internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument, AddComposableNode, SetLaunchArgument
from launch_utils.launch.conditions import MultipleIfConditions

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.events import Shutdown

from ament_index_python import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(ld, "input/vehicle_inputs", "/control/vehicle_inputs")
    AddLaunchArgument(ld, "output/time", "/clock")
    AddLaunchArgument(ld, "output/vehicle", "/vehicle/state")
    AddLaunchArgument(ld, "output/camera", "/sensing/front_facing_camera/raw")
    AddLaunchArgument(ld, "ip", "")
    AddLaunchArgument(ld, "hostname", "")
    AddLaunchArgument(ld, "use_sim_time", "True")

    # -----
    # Nodes
    # -----

    AddComposableNode(
        ld,
        plugin="chrono::ros::ChROSBridge",
        executable="chrono_ros_bridge_node",
        package="chrono_ros_bridge",
        name="chrono_ros_bridge",
        remappings=[
            ("~/input/driver_inputs", GetLaunchArgument("input/vehicle_inputs")),
            ("~/output/time", GetLaunchArgument("output/time")),
            ("~/output/vehicle", GetLaunchArgument("output/vehicle")),
            ("~/output/camera/front_facing_camera", GetLaunchArgument("output/camera")),
        ],
        parameters=[
             {"use_sim_time": GetLaunchArgument("use_sim_time")},
             {"ip": GetLaunchArgument("ip")},
             {"hostname": GetLaunchArgument("hostname")},
        ],
        node_kwargs={
          "output": "screen",
        },
    )

    return ld

