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
from launch.substitutions import LaunchConfiguration, PythonExpression

# internal imports
from launch_utils import AddLaunchArgument, GetLaunchArgument, GetPackageSharePath
from enum import Enum

def generate_launch_description():

    robot_ns = LaunchConfiguration('robot_ns')

    ekf_config = GetPackageSharePath(
        "art_localization_launch", "config", "ekf_config.yaml"
    )
    
    gps = PythonExpression(['"', robot_ns, "/gps", '"'])
    imu = PythonExpression(['"', robot_ns, "/imu", '"'])
    base_link = PythonExpression(['"', robot_ns, "/base_link", '"'])
    odom = PythonExpression(['"', robot_ns, "/odom", '"'])

    # Transforms
    gps_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gps_transform",
        arguments=["0", "0", "0", "0", "0", "0", base_link, gps],
    )
    imu_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_transform",
        arguments=["0", "0", "0", "3.1416", "-1.5708", "0", base_link, imu],
    )
    odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", odom, base_link],
    )

    # Navsat Transform
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        namespace=robot_ns,
        respawn=True,
        parameters=[
            {
                "magnetic_declination_radians": -0.0523599,
                "yaw_offset": 0.0,
                "zero_altitude": True,
                "use_odometry_yaw": True,
                "wait_for_datum": False,
                #"use_local_cartesian": False,
                "publish_filtered_gps": True,
                "broadcast_cartesian_transform": False,
            }
        ],
        remappings=[
            ("/imu/data", PythonExpression(['"', '/', robot_ns, "/imu/data", '"'])),
            (PythonExpression(['"', '/', robot_ns, "/gps/fix", '"']), PythonExpression(['"', '/', robot_ns, "/output/gps/fix", '"'])),
            ('/odometry/filtered', PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
            ('/odometry/gps', PythonExpression(['"', '/', robot_ns, "/odometry/gps", '"'])),
            ('/gps/filtered', PythonExpression(['"', '/', robot_ns, "/gps/filtered", '"'])),
            # ('/odometry/filtered', '/odometry/gps')
        ],
    )

    # Set Datum Node
    set_datum_client = Node(
        package='navsat_util',
        executable='datum_service',
        name="set_datum_node",
        namespace=robot_ns,
        parameters =[ {
            "namespace": robot_ns
        }]
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        # namespace=robot_ns,
        parameters=[
            ekf_config,
            {
                'map_frame': "map",
                'odom_frame': odom,
                'base_link_frame': base_link,
                'world_frame': 'map'
            }
        ],
        remappings=[
            ("/odometry/filtered", PythonExpression(['"', '/', robot_ns, "/odometry/filtered", '"'])),
            ("/imu/data", PythonExpression(['"', '/', robot_ns, "/imu/data", '"'])),
            ("/odometry/gps", PythonExpression(['"', '/', robot_ns, "/odometry/gps", '"'])),
            ("/gps/filtered", PythonExpression(['"', '/', robot_ns, "/gps/filtered", '"'])),
        ],
    )

    return LaunchDescription(
        [
            gps_transform,
            imu_transform,
            set_datum_client,
            odom_transform,
            navsat_transform_node,
            ekf_filter_node,
        ]
    )
