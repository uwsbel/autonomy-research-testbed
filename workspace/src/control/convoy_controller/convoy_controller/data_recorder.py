import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RosbagRecordPlaybackNode(Node):
    def __init__(self):
        super().__init__('rosbag_record_playback_node')
        self.mode = None
        self.declare_parameter('mode', 'record')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.bag_prefix = self.mode

        if self.mode == 'record':
            self.get_logger().info('Mode: Record')
            self.start_recording()
        elif self.mode == 'playback':
            self.get_logger().info('Mode: Playback')
            self.start_playback()
        else:
            self.get_logger().error('Unknown mode. Please use "record" or "playback".')

    def start_recording(self):
        # Start recording the topics in a rosbag
        bag_name = f'{self.bag_prefix}_rosbag'
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
        bag_name = f'{self.bag_prefix}_rosbag'
        playback_command = [
            'ros2', 'bag', 'play', f'{self.bag_prefix}_rosbag'
        ]
        record_command = [
            'ros2', 'bag', 'record',
            '/artcar_1/odometry/filtered',
            '-o', f'{self.bag_prefix}_rosbag'
        ]

        self.get_logger().info(f'Starting playback from rosbag: {bag_name}')
        subprocess.Popen(playback_command)

        self.get_logger().info(f'Starting recording: {bag_name}')
        subprocess.Popen(record_command)

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecordPlaybackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
