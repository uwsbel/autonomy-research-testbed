from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='simulation',
        #     namespace='',
        #     executable='chrono',
        #     name='chrono',
        #     parameters=[
        #         {"use_sim_time": True},
        #         {"step_size": 1e-3},
        #         {"cone_offset_x": 0.0},
        #         {"cone_offset_y": 0.25},
        #         {"init_loc_x": -2.5},
        #         {"init_loc_y": 0.5},
        #         {"init_angle_z": 1.57},
        #         {"cones_from_file": True},
        #         {"cone_path_file": "data/paths/cone_paths_2.csv"},
        #         {"num_cones":50},
        #         {"save_sensor_data": False},
        #         {"create_semantic_maps":False},
        #         {"sensor_output_dir":"sensor_output/"},
        #     ],
        #     output='screen',
        #     emulate_tty=True,
        #     arguments=[('__log_level:=debug')]
        # ),
        Node(
            package='cone_detector',
            namespace='',
            executable='object_recognition',
            name='object_recognition',
            parameters=[
                 {"use_sim_time": True},
                 {"model":"data/model_refined"},
                 {"camera_calibration_file":"data/calibration.json"},
                 {"vis": True}
            ]
        ),
        Node(
            package='path_planning',
            namespace='',
            executable='path_planning',
            name='path_planning',
            parameters=[
                {"vis": True},
                {"lookahead": .75}
            ]
        ),
        Node(
            package='control',
            namespace='',
            executable='pid',
            name='pid',
            parameters=[
                 {"control_mode": "PID"},
                 {"control_file": "data/smallest_radius_right.csv"},
                 {"steering_gain": 1.6},
                 {"throttle_gain": 0.08},
            ]
        )
    ])
