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
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer

# internal imports
from launch_utils import (
    IncludeLaunchDescriptionWithCondition,
    GetLaunchArgument,
    AddLaunchArgument,
    SetLaunchArgument,
)


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(ld, "use_sim", "False")
    AddLaunchArgument(ld, "art_oak", "True")
    AddLaunchArgument(ld, "use_sim_time", "False")
    AddLaunchArgument(ld, "art_oak", "True")
    SetLaunchArgument(
        ld, "use_sim_time", "True", condition=IfCondition(GetLaunchArgument("use_sim"))
    )

    SetLaunchArgument(
        ld,
        "disable_art_sensing",
        "True",
        condition=IfCondition(GetLaunchArgument("use_sim")),
    )
    SetLaunchArgument(
        ld,
        "disable_art_vehicle",
        "True",
        condition=IfCondition(GetLaunchArgument("use_sim")),
    )
    SetLaunchArgument(
        ld,
        "disable_art_simulation",
        "True",
        condition=UnlessCondition(GetLaunchArgument("art_oak")),
    )
    SetLaunchArgument(
        ld,
        "disable_ekf_estimation",
        "True",
        condition=IfCondition(GetLaunchArgument("art_oak")),
    )
    SetLaunchArgument(
        ld,
        "disable_particle_filter_estimation",
        "True",
        condition=IfCondition(GetLaunchArgument("art_oak")),
    )
    SetLaunchArgument(
        ld,
        "disable_centerline_objects_path_planner",
        "True",
        condition=IfCondition(GetLaunchArgument("art_oak")),
    )
    SetLaunchArgument(
        ld,
        "disable_usb_cam",
        "True",
        condition=IfCondition(GetLaunchArgument("art_oak")),
    )
    SetLaunchArgument(
        ld,
        "disable_xsens",
        "True",
        condition=IfCondition(GetLaunchArgument("art_oak")),
    )

    IncludeLaunchDescriptionWithCondition(ld, "art_launch", "art")

    return ld
