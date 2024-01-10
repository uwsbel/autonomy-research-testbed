import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import csv
import time
import math

from sbg_driver.msg import SbgGpsVel  # Import the correct message type
from chrono_ros_interfaces.msg import Body

class GPSVelocitySubscriber(Node):
    def __init__(self):
        super().__init__('gps_velocity_subscriber')
        self.gps_velocity_data = []  # To store (timestamp, velocity.x) tuples
        self.subscription = self.create_subscription(
            Body,
            '/chrono_ros_node/output/vehicle/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.start_time = time.time()

    def listener_callback(self, msg):
        
        timestamp = elapsed_time = time.time() - self.start_time 
        velocity_x = msg.twist.linear.x  # Extracting the x-component of velocity
        #print(velocity_x)

        self.gps_velocity_data.append((timestamp, velocity_x))

class IMUSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.imu_data = []  # To store (timestamp, acceleration) tuples
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.start_time = time.time()
        

    def listener_callback(self, msg):
        timestamp = elapsed_time = time.time() - self.start_time 
        acceleration = msg.linear_acceleration.x  # Change to .y or .z if needed
        self.imu_data.append((timestamp, acceleration))

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.odom_data = []  # To store (timestamp, velocity) tuples
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.start_time = time.time()

    def listener_callback(self, msg):
        timestamp = elapsed_time = time.time() - self.start_time 
        velocity = -msg.twist.twist.linear.x  
        velocity_y = msg.twist.twist.linear.y
        velcoity_z = msg.twist.twist.linear.y# Change to .y or .z if needed
        speed = math.sqrt(velocity**2 + velocity_y**2 )
        
        self.odom_data.append((timestamp, velocity, velocity_y, speed))

def integrate_acceleration(acceleration_data):
    velocity = [0]  # Initial velocity is assumed to be 0
    for i in range(1, len(acceleration_data)):
        time_delta = acceleration_data[i][0] - acceleration_data[i-1][0]
        velocity.append(velocity[-1] + 0.5 * (acceleration_data[i-1][1] + acceleration_data[i][1]) * time_delta)
    return velocity

def save_data_to_csv(filename, data):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Time (state-estimator)', 'Time (GT)', 'Vel (GT)', 'Vel (state-estimator))'])
        for row in data:
            writer.writerow(row)

def save_combined_data_to_txt(filename, odom_data, gps_data):
    # Ensure both lists have the same length
    min_length = min(len(odom_data), len(gps_data))

    # Truncate lists to the same length
    odom_data = odom_data[:min_length]
    gps_data = gps_data[:min_length]

    # Open the file and write the data
    with open(filename, mode='w', newline='') as file:
        for odom, gps in zip(odom_data, gps_data):
            file.write(f"{odom[1]}\t{gps[1]}\n")

def main(args=None):
    rclpy.init(args=args)

    imu_subscriber = IMUSubscriber()
    odom_subscriber = OdomSubscriber()
    gps_velocity_subscriber = GPSVelocitySubscriber()

    start_time = time.time() 

    try:
        while rclpy.ok():
            rclpy.spin_once(imu_subscriber, timeout_sec=0.1)
            rclpy.spin_once(odom_subscriber, timeout_sec=0.1)
            rclpy.spin_once(gps_velocity_subscriber, timeout_sec=0.1)

            current_time = time.time()
            if current_time - start_time > 20:  # Check if 25 seconds have passed
                break  # Exit

    except KeyboardInterrupt:
        pass

    combined_data = []
    for odom, gps_vel in zip(odom_subscriber.odom_data, gps_velocity_subscriber.gps_velocity_data):
        timestep = odom[0]
        timestep_se = gps_vel[0]
        ground_truth_velocity = gps_vel[1]
        state_estimator_velocity = odom[1] 
        combined_data.append([timestep, timestep_se, ground_truth_velocity, state_estimator_velocity])

    # Save combined data to CSV
    save_data_to_csv('sim_velocity_data_9.csv', combined_data)


#     start_time = odom_subscriber.odom_data[0][0] if odom_subscriber.odom_data else 0
#     odom_indices = [(t[0] - start_time) for t in odom_subscriber.odom_data]
#     gps_times = [(t[0] - start_time) for t in gps_velocity_subscriber.gps_velocity_data]

    odom_indices = range(len(odom_subscriber.odom_data))
    gps_indices = range(len(gps_velocity_subscriber.gps_velocity_data))

    plt.rcParams.update({
    'axes.titlesize': 20,
    'axes.labelsize': 18,
    'lines.linewidth': 1.5,
    'lines.markersize': 6,
    'xtick.labelsize': 16,
    'ytick.labelsize': 16,
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Palatino', 'serif'],
    # "font.serif" : ["Computer Modern Serif"],
})


    plt.figure(figsize=(10, 6))  # You can adjust the size as needed


    save_combined_data_to_txt('combined_data.txt', odom_subscriber.odom_data, gps_velocity_subscriber.gps_velocity_data)


    
    plt.plot([d[0] for d in odom_subscriber.odom_data], [d[1] for d in odom_subscriber.odom_data], label='State-Estimator Velocity')
    #plt.plot([d[0] for d in odom_subscriber.odom_data], [d[2] for d in odom_subscriber.odom_data], label='State-Estimator Velocity (y)')
    #plt.plot([d[0] for d in odom_subscriber.odom_data], [d[3] for d in odom_subscriber.odom_data], label='Speed')
    plt.plot([d[0] for d in gps_velocity_subscriber.gps_velocity_data], [d[1] for d in gps_velocity_subscriber.gps_velocity_data], label='Ground-Truth Velocity')


    # Add labels, title, and legend
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Comparison in Simulation')
    
    #plt.ylim(0, 0.20)

    plt.legend(loc='upper left', bbox_to_anchor=(1, 1))
    plt.tight_layout()
    plt.grid(True)

    # Display the plot
    plt.show()

    # Save the figure
    plt.savefig('combined_velocity.png')

if __name__ == '__main__':
    main()
