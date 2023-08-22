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
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------
            
    AddLaunchArgument(ld, "art_localization/input/vehicle_inputs", "/control/vehicle_inputs")
    AddLaunchArgument(ld, "art_localization/output/vehicle_state", "/vehicle/state")
    #TODO: might want ot use multipleIfConditions here
    if(IfCondition(PythonExpression([(GetLaunchArgument("use_sim"))]))):
        AddLaunchArgument(ld, "art_localization/input/gps", "/chrono_ros_bridge/output/gps/data")
        AddLaunchArgument(ld, "art_localization/input/magnetometer", "/chrono_ros_bridge/output/magnetometer/data")
        AddLaunchArgument(ld, "art_localization/input/groundTruth", "/vehicle/state")
        AddLaunchArgument(ld, "art_localization/input/gyroscope","/chrono_ros_bridge/output/gyroscope/data")
        AddLaunchArgument(ld, "art_localization/input/accelerometer","/chrono_ros_bridge/output/accelerometer/data")
    else:
        AddLaunchArgument(ld,"art_localization/input/gps","/sensing/gps/data")
        AddLaunchArgument(ld, "art_localization/input/magnetometer","/sensing/magnetometer/data")
        #TODO: Ground truth from RTK
        AddLaunchArgument(ld, "art_localization/input/groundTruth", "/vehicle/state")

        AddLaunchArgument(ld, "art_localization/input/gyroscope", "/sensing/gyroscope/data")
        AddLaunchArgument(ld, "art_localization/input/accelerometer", "/sensing/accelerometer/data")

    AddLaunchArgument(ld, "SE_mode", "GT")
    AddLaunchArgument(ld, "use_sim_time", "False")
    AddLaunchArgument(ld, "print_csv", "False")

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
                ("~/input/vehicleInput", GetLaunchArgument("art_localization/input/vehicle_inputs")),
                ("~/output/vehicle_state", GetLaunchArgument("art_localization/output/vehicle_state")),
                ("~/input/groundTruth", GetLaunchArgument("art_localization/input/groundTruth")),
        ],
        parameters=[
            {"SE_mode": GetLaunchArgument("SE_mode")},
            {"use_sim_time": GetLaunchArgument("use_sim_time")},
            {"print_csv": GetLaunchArgument("print_csv")}
        ]
    )
    ld.add_action(node)

    return ld 
