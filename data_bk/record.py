import os
import subprocess
import sys
import rclpy
from rclpy.node import Node


class RosbagRecordPlaybackNode(Node):
    def __init__(self, mode, trial_number):
        super().__init__('rosbag_record_playback_node')
        self.mode = mode
        self.trial_number = trial_number
        self.trial_folder = f'trial_{self.trial_number}'
        
        # Create the trial-specific subfolder if it doesn't exist
        if not os.path.exists(self.trial_folder):
            os.makedirs(self.trial_folder)

        self.bag_prefix = f'{self.trial_folder}/{self.mode}_rosbag'

        if self.mode == 'record':
            self.get_logger().info(f'Mode: Record for Trial {self.trial_number}')
            self.start_recording()
        elif self.mode == 'playback':
            self.get_logger().info(f'Mode: Playback for Trial {self.trial_number}')
            self.start_playback()
        else:
            self.get_logger().error('Unknown mode. Please use "record" or "playback".')

    def start_recording(self):
        # Start recording the topics in a rosbag in the trial-specific folder
        bag_name = f'{self.trial_folder}/record_rosbag'
        command = [
            'ros2', 'bag', 'record',
            '/artcar_1/control/vehicle_inputs',
            '/artcar_1/odometry/filtered',
            '-o', bag_name
        ]
        self.get_logger().info(f'Starting rosbag recording: {bag_name}')
        subprocess.Popen(command)

    def start_playback(self):
        # Start playback of /artcar_1/control/vehicle_inputs and record /artcar_1/odometry/filtered
        playback_bag_name = f'{self.trial_folder}/record_rosbag'  # Use the recorded rosbag
        new_record_bag_name = self.get_unique_bag_name(f'{self.trial_folder}/playback_odometry_rosbag')

        # Play the control inputs from the previous rosbag
        playback_command = [
            'ros2', 'bag', 'play', playback_bag_name, '--storage', 'sqlite3'
        ]
        # Record the new /artcar_1/odometry/filtered data into a separate rosbag
        record_command = [
            'ros2', 'bag', 'record',
            '/artcar_1/odometry/filtered',
            '/artcar_1/control/vehicle_inputs',
            '-o', new_record_bag_name, '--storage', 'sqlite3'
        ]

        self.get_logger().info(f'Starting playback from rosbag: {playback_bag_name}')
        playback_process = subprocess.Popen(playback_command)

        self.get_logger().info(f'Starting recording odometry to: {new_record_bag_name}')
        record_process = subprocess.Popen(record_command)

        # Wait for the playback process to complete
        playback_process.wait()

        self.get_logger().info('Playback finished. Stopping the recording process.')
        
        # Once playback is over, terminate the recording process
        record_process.terminate()

        self.get_logger().info('Recording process terminated.')

    def get_unique_bag_name(self, base_name):
        """
        Generate a unique bag name by appending _1, _2, etc., if the file already exists.
        """
        counter = 1
        new_bag_name = base_name
        while os.path.exists(f'{new_bag_name}'):  # rosbag2 sqlite3 creates _0.db3
            new_bag_name = f'{base_name}_{counter}'
            counter += 1
        return new_bag_name


def main(args=None):
    if len(sys.argv) != 3:
        print("Usage: python3 script_name.py [record|playback] [trial_number]")
        sys.exit(1)

    mode = sys.argv[1]
    trial_number = sys.argv[2]

    if mode not in ['record', 'playback']:
        print("Invalid mode. Please choose 'record' or 'playback'.")
        sys.exit(1)

    rclpy.init(args=args)
    node = RosbagRecordPlaybackNode(mode, trial_number)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
