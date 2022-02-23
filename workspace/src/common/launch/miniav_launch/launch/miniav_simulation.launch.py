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
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.events import Shutdown

from ament_index_python import get_package_share_directory


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

    AddLaunchArgument("input/vehicle_inputs", "/control/vehicle_inputs")
    AddLaunchArgument("output/time", "/clock")
    AddLaunchArgument("output/vehicle", "/vehicle/state")
    AddLaunchArgument("output/camera", "/sensing/front_facing_camera/raw")
    AddLaunchArgument("ip", "")
    AddLaunchArgument("hostname", "")
    AddLaunchArgument("use_sim_time", "True")

    # ------------
    # Launch Files
    # ------------

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('miniav_launch'),
                'launch/miniav_stack.launch.py')),
        launch_arguments=[
            ('use_sim_time', 'True'),
            ('use_sim_msg', 'True'),
        ]
    )
    launch_description.add_action(stack)

    # -----
    # Nodes
    # -----

    chrono_ros_bridge = Node(
        package='chrono_ros_bridge',
        namespace='',
        executable='chrono_ros_bridge',
        name='chrono_ros_bridge',
        remappings=[
            ("~/input/driver_inputs", LaunchConfiguration("input/vehicle_inputs")),
            ("~/output/time", LaunchConfiguration("output/time")),
            ("~/output/vehicle", LaunchConfiguration("output/vehicle")),
            ("~/output/camera/front_facing_camera", LaunchConfiguration("output/camera")),
        ],
        parameters=[
             {"use_sim_time": LaunchConfiguration("use_sim_time")},
             {"ip": LaunchConfiguration("ip")},
             {"hostname": LaunchConfiguration("hostname")},
        ],
        on_exit=EmitEvent(event=Shutdown()),
		
    )
    launch_description.add_action(chrono_ros_bridge)

    return launch_description

