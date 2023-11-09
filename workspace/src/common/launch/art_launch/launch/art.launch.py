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
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration, Shutdown, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory as FindPackageShare

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
    AddLaunchArgument(ld, "use_sim_time", "False")
    SetLaunchArgument(ld, "use_sim_time", "True", condition=IfCondition(GetLaunchArgument("use_sim")))
   
    AddLaunchArgument(ld, "container", "") 
    container_name = AddLaunchArgument(ld, "container_name", "art_container") 

    SetLaunchArgument(ld, "disable_art_sensing", "True", condition=IfCondition(GetLaunchArgument("use_sim")))
    SetLaunchArgument(ld, "disable_art_vehicle", "True", condition=IfCondition(GetLaunchArgument("use_sim")))
    SetLaunchArgument(ld, "disable_art_simulation", "True", condition=UnlessCondition(GetLaunchArgument("use_sim")))

    # -------------
    # Composability
    # -------------

    # If composability is desired, all included launch descriptions should attach to this container and use intraprocess communication

    use_composability = IfCondition(AddLaunchArgument(ld, "use_composability", "False"))

    # If a container name is not provided,
    # set the name of the container launched above for image_proc nodes
    set_container_name = SetLaunchConfiguration(
        condition=use_composability,
        name='container',
        value=container_name
    )
    ld.add_action(set_container_name)

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[],
        condition=use_composability,
        output="screen",
    )
    ld.add_action(container)

    # move_broly_node = Node(
    #     package='control',  # Replace with your actual package
    #     executable='control',       # Replace with your actual node executable
    #     name='move_robot',
    #     output='screen',
    #     parameters=[{'throttle': 1.0}]  # Replace with actual parameters for throttle
    # )
    # ld.add_action(move_broly_node)

    # Timer action to stop the robot after 20 seconds
    stop_robot_timer = TimerAction(
        period=30.0,
        actions=[Shutdown()]
    )
    ld.add_action(stop_robot_timer)

    # Include the sbg_driver launch file
    #sbg_driver_launch_file = FindPackageShare('sbg_driver').find('sbg_driver') + '/launch/sbg_device_launch.py'
    sbg_driver_launch_file = FindPackageShare('sbg_driver') + '/launch/sbg_device_launch.py'

    sbg_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sbg_driver_launch_file),
        launch_arguments={'param': 'value'}.items()  # Replace with actual parameters
    )
    ld.add_action(sbg_driver_launch)

    nmea_navsat_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='nmea_serial_driver',
        output='screen'
    )
    ld.add_action(nmea_navsat_driver_node)


    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(ld, "art_control_launch", "art_control")
    #IncludeLaunchDescriptionWithCondition(ld, "art_sensing_launch", "art_sensing")
    IncludeLaunchDescriptionWithCondition(ld, "art_vehicle_launch", "art_vehicle")

    return ld 
