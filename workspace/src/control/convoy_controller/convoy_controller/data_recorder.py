import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from art_msgs.msg import VehicleInput  # Replace with the actual message type for your topic
import csv
from datetime import datetime

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')
        
        # Create subscriptions
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/artcar_1/odometry/filtered',
            self.odom_callback,
            10
        )
        
        self.throttle_subscription = self.create_subscription(
            VehicleInput,
            '/artcar_1/control/vehicle_inputs',
            self.throttle_callback,
            10
        )
        
        # Initialize storage for data
        self.data = []

        # Set up CSV file name with timestamp
        self.csv_file = f'data_record_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        
        # Ensure that the CSV has the headers initially
        self.create_csv()
        
    def create_csv(self):
        # Create a CSV file with headers
        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'acceleration_x', 'acceleration_y', 'acceleration_z', 'throttle'])
    
    def odom_callback(self, msg: Odometry):
        # Extract acceleration from the odometry message
        acceleration_x = msg.twist.twist.linear.x
        acceleration_y = msg.twist.twist.linear.y
        acceleration_z = msg.twist.twist.linear.z
        
        # Add data with timestamp to storage
        self.data.append({
            'timestamp': self.get_clock().now().to_msg().sec,
            'acceleration_x': acceleration_x,
            'acceleration_y': acceleration_y,
            'acceleration_z': acceleration_z,
            'throttle': None  # Placeholder, will be updated when throttle data arrives
        })

    def throttle_callback(self, msg: VehicleInput):
        # Extract throttle value from the message
        throttle = msg.throttle
        
        # Find the latest entry in self.data that has no throttle value and update it
        for entry in reversed(self.data):
            if entry['throttle'] is None:
                entry['throttle'] = throttle
                break
        
        # Periodically save the data to CSV
        if len(self.data) >= 10:  # Save every 10 data points for example
            self.save_to_csv()

    def save_to_csv(self):
        # Write data to CSV
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            for entry in self.data:
                writer.writerow([entry['timestamp'], entry['acceleration_x'], entry['acceleration_y'], entry['acceleration_z'], entry['throttle']])
        
        # Clear the saved data from memory
        self.data.clear()

def main(args=None):
    rclpy.init(args=args)
    
    data_recorder = DataRecorder()
    
    rclpy.spin(data_recorder)
    
    data_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
