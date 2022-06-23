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
from launch.actions import ExecuteProcess

# skipping CompressedDepth, doesn't appear to work properly
# /image_raw/theora adds the greatest time increase to Col2Perc
image_topics = ['/sensing/front_facing_camera/raw', '/image_raw/compressed', '/image_raw/compressedDepth', '/camera_info']
imu_topics = ['/gnss', '/imu/acceleration', '/imu/angular_velocity', '/imu/data', '/imu/dq', '/imu/dv', '/imu/mag', '/imu/time_ref']
vehicle_topics = ['/control/vehicle_inputs']

all_topics = image_topics + imu_topics + vehicle_topics

# This can be launched from the command
#
#    ros2 launch art_launch art_rosbag_launch.launch.py
#
# and will record all the ros topics which are needed for testing
def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd = (['ros2', 'bag', 'record'] + all_topics),
            output = 'screen'
        )
    ])















