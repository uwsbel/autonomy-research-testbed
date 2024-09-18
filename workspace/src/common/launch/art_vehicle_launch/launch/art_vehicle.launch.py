# ros imports
from launch import LaunchDescription

import os
# internal imports
from launch_utils import IncludeLaunchDescriptionWithCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    veh_config_env = os.getenv('VEH_CONFIG', 'default')
    veh_config_file = f'{veh_config_env}.yaml'

    declare_veh_config_arg = DeclareLaunchArgument(
        'veh_config_file',
        default_value=veh_config_file,
        description='Name of the vehicle configuration file without path'
    )
    ld.add_action(declare_veh_config_arg)

    # ---------------
    # Launch Includes
    # ---------------

    IncludeLaunchDescriptionWithCondition(ld, "art_vehicle_launch", "arduino_driver")

    return ld
