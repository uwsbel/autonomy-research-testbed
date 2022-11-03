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



    '''
    Copied from the RovioNode.hpp file, does not reflect other files -- not complete
    '''

    #subscribers
    AddLaunchArgument("input/imu", "/sensing/imu/raw")   # not sure if this is right need to find right info for imu
    AddLaunchArgument("input/image", "/sensing/front_facing_camera/raw")

    # # not sure if needed but: -- I think it is used for validation
    # AddLaunchArgument("input/Groundtruth", "/sensing/Groundtruth") # true position from simulation
    # AddLaunchArgument("input/GroundtruthOdometry", "/sensing/GroundtruthOdometry") # 
    # AddLaunchArgument("input/velocity", "/sensing/veloctiy") # true velocity

    # AddLaunchArgument("input/vehicle_state", "/vehicle/state")

    #publishers
    AddLaunchArgument("output/Odometry", "/vio/Odometry") # pos and velocity
    AddLaunchArgument("output/Transform", "/vio/Transform") # 
    AddLaunchArgument("output/PoseWithCovStamped", "/vio/PoseWithCovStamped") #pos with guasian
    AddLaunchArgument("output/pub_T_J_W_transform", "/vio/pub_T_J_W_transform")  # rename - not sure what this is
    AddLaunchArgument("output/Pcl", "/vio/Pcl")
    AddLaunchArgument("output/Patch", "/vio/Patch")
    AddLaunchArgument("output/Markers", "/vio/Markers")
    # ros::Publisher pubExtrinsics_[mtState::nCam_];  # not sure how to replace
    AddLaunchArgument("output/ImuBias", "/vio/ImuBias")

    # AddLaunchArgument("output/objects", "/perception/objects")



    AddLaunchArgument("use_sim_time", "False")
    AddLaunchArgument("model", "data/model_refined")
    AddLaunchArgument("camera_calibration_file", "data/calibration.json")
    AddLaunchArgument("vis", "False")

    # -----
    # Nodes
    # -----

    node = Node(
        package='rovio',
        namespace='',
        executable='rovio_node',
        name='rovio_node',
        remappings=[
            ("~/input/imu", LaunchConfiguration("input/imu")),
            ("~/input/image", LaunchConfiguration("input/image")),
            # ("~/input/Groundtruth", LaunchConfiguration("input/Groundtruth")),
            # ("~/input/GroundtruthOdometry", LaunchConfiguration("input/GroundtruthOdometry")),
            # ("~/input/velocity", LaunchConfiguration("input/velocity")),
            ("~/output/Odometry", LaunchConfiguration("output/Odometry")),
            ("~/output/Transform", LaunchConfiguration("output/Transform")),

            ("~/output/PoseWithCovStamped", LaunchConfiguration("output/PoseWithCovStamped")),
            ("~/output/pub_T_J_W_transform", LaunchConfiguration("output/pub_T_J_W_transform")),
            ("~/output/Pcl", LaunchConfiguration("output/Pcl")),
            ("~/output/Patch", LaunchConfiguration("output/Patch")),
            ("~/output/Markers", LaunchConfiguration("output/Markers")),
            ("~/output/ImuBias", LaunchConfiguration("output/ImuBias"))

            # missing one check above

            # ("~/input/vehicle_state", LaunchConfiguration("input/vehicle_state")),
            # ("~/output/objects", LaunchConfiguration("output/objects"))
        ],
        parameters=[
             {"use_sim_time": LaunchConfiguration("use_sim_time")},
             {"model":LaunchConfiguration("model")},
             {"camera_calibration_file":LaunchConfiguration("camera_calibration_file",)},    #might need to change this
             {"vis": LaunchConfiguration("vis")}
        ]
    )
    launch_description.add_action(node)

    return launch_description
