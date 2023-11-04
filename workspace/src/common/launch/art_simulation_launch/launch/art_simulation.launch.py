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

# ros imports
from launch import LaunchDescription
from launch_ros.actions import SetRemap

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()

    # ----------------
    # Launch Arguments
    # ----------------

    AddLaunchArgument(
        ld, "art_simulation/input/vehicle_inputs", "/control/vehicle_inputs"
    )
    AddLaunchArgument(ld, "art_simulation/output/gps", "/sensing/gps/data")
    AddLaunchArgument(ld, "art_simulation/output/gyroscope", "/sensing/gyroscope/data")
    AddLaunchArgument(
        ld, "art_simulation/output/accelerometer", "/sensing/accelerometer/data"
    )
    AddLaunchArgument(
        ld, "art_simulation/output/magnetometer", "/sensing/magnetometer/data"
    )
    AddLaunchArgument(
        ld, "art_simulation/output/camera", "/sensing/front_facing_camera/raw"
    )

    # ------
    # Remaps
    # ------

    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/input/driver_inputs",
            dst=GetLaunchArgument("art_simulation/input/vehicle_inputs"),
        )
    )
    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/output/acc/data",
            dst=GetLaunchArgument("art_simulation/output/accelerometer"),
        )
    )
    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/output/cam/data",
            dst=GetLaunchArgument("art_simulation/output/camera"),
        )
    )
    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/output/gps/data",
            dst=GetLaunchArgument("art_simulation/output/gps"),
        )
    )
    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/output/gyro/data",
            dst=GetLaunchArgument("art_simulation/output/gyroscope"),
        )
    )
    ld.add_action(
        SetRemap(
            src="/chrono_ros_node/output/mag/data",
            dst=GetLaunchArgument("art_simulation/output/magnetometer"),
        )
    )

    return ld
