import rclpy
from rclpy.node import Node
from art_msgs.msg import VehicleInput  # Replace with the actual message type for your topic
import csv

class DataPlayback(Node):
    def __init__(self, csv_file):
        super().__init__('data_playback')

        # Create a publisher to publish vehicle inputs
        self.publisher_ = self.create_publisher(VehicleInput, '/artcar_1/control/vehicle_inputs', 10)

        # Load data from the CSV
        self.data = self.load_csv(csv_file)
        self.get_logger().info(f'Loaded {len(self.data)} records from CSV file: {csv_file}')
        
        # Start playback from the first record
        self.playback_index = 0
        self.first_timestamp = None
        self.playback_start_time = self.get_clock().now()

        # Call the playback callback for the first time
        self.playback_callback()

    def load_csv(self, csv_file):
        """ Load the recorded data from the CSV file """
        data = []
        with open(csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                data.append({
                    'timestamp': int(row['timestamp']),
                    'throttle': float(row['throttle']),
                    'steering_angle': float(row['steering_angle']),
                })
        return data

    def playback_callback(self):
        """ Playback callback that manages real-time pacing and publication """
        if self.playback_index >= len(self.data):
            self.get_logger().info("Playback Complete.")
            return

        current_row = self.data[self.playback_index]

        # Calculate the elapsed real time since the start of playback
        current_time = self.get_clock().now()
        elapsed_real_time = (current_time - self.playback_start_time).nanoseconds

        # Calculate the expected elapsed time from the data timestamps
        if self.first_timestamp is None:
            self.first_timestamp = current_row['timestamp']

        elapsed_data_time_ns = (current_row['timestamp'] - self.first_timestamp) * 1e6  # convert ms to ns

        # If real time has caught up to or is ahead of data time, publish the message
        if elapsed_real_time >= elapsed_data_time_ns:
            # Publish the message
            vehicle_input = VehicleInput()
            vehicle_input.throttle = current_row['throttle']
            vehicle_input.steering = current_row['steering_angle']

            self.publisher_.publish(vehicle_input)

            # Log the playback
            self.get_logger().info(f"Playing back: Throttle = {vehicle_input.throttle}, Steering = {vehicle_input.steering}")

            # Move to the next row
            self.playback_index += 1

            # Calculate time difference to next row (to avoid overshooting)
            if self.playback_index < len(self.data):
                next_row = self.data[self.playback_index]
                next_timestamp_diff = (next_row['timestamp'] - current_row['timestamp']) * 1e6  # ms to ns
                next_interval_sec = next_timestamp_diff / 1e9  # Convert ns to seconds
            else:
                next_interval_sec = 0.01  # Default small value if no next row

        else:
            # If real time has not caught up yet, set a very short delay (check again soon)
            next_interval_sec = 0.001  # 1 ms delay to check again

        # Schedule the next call to playback_callback dynamically
        self.timer = self.create_timer(next_interval_sec, self.playback_callback)

def main(args=None):
    rclpy.init(args=args)

    # Set the CSV file name to play back from
    csv_file = 'data_record_20240906_183144.csv'  # Replace with actual file name or pass as argument
    data_playback = DataPlayback(csv_file)

    rclpy.spin(data_playback)

    data_playback.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
