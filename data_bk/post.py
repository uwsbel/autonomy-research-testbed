import rclpy
import rosbag2_py
import csv
import argparse
import glob
import os
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def process_rosbag(bag_path, output_csv):
    # Initialize the ROS2 system
    rclpy.init()

    # Setup the bag reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Define topics
    control_topic = '/artcar_1/control/vehicle_inputs'
    odometry_topic = '/artcar_1/odometry/filtered'

    # Deserialized messages for the two topics
    control_msgs = []
    odometry_msgs = []

    # Get the topic types from the bag
    topics_and_types = reader.get_all_topics_and_types()
    type_dict = {topic.name: topic.type for topic in topics_and_types}

    # Read through the bag and extract relevant messages
    while reader.has_next():
        (topic, data, t) = reader.read_next()

        # Deserialize based on topic
        msg_type = get_message(type_dict[topic])
        msg = deserialize_message(data, msg_type)

        # Filter by the control and odometry topics
        if topic == control_topic:
            control_msgs.append((t, msg))
        elif topic == odometry_topic:
            odometry_msgs.append((t, msg))

    # Sort the control and odometry messages by timestamp (just in case they are not in order)
    control_msgs.sort(key=lambda x: x[0])
    odometry_msgs.sort(key=lambda x: x[0])

    # Function to find the closest odometry for each control input
    def find_closest_odometry(control_msgs, odometry_msgs):
        matched_data = []
        odometry_index = 0

        for control_time, control in control_msgs:
            # Find the latest odometry message with a timestamp less than or equal to the control timestamp
            while odometry_index < len(odometry_msgs) and odometry_msgs[odometry_index][0] <= control_time:
                odometry_index += 1

            # Use the last odometry data found (that has a timestamp <= control input)
            if odometry_index > 0:
                matched_odometry = odometry_msgs[odometry_index - 1]
                matched_data.append((control_time, control, matched_odometry[1]))

        return matched_data

    # Get the matched data
    matched_data = find_closest_odometry(control_msgs, odometry_msgs)

    # Write the matched data to a CSV file
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Write header
        writer.writerow(["control_timestamp", "control_steering", "control_throttle",
                         "odometry_pose_x", "odometry_pose_y", "odometry_pose_theta",
                         "odometry_velocity_linear", "odometry_velocity_angular"])

        # Write the data rows
        for control_time, control, odometry in matched_data:
            # Extract specific fields from the control message
            control_steering = control.steering
            control_throttle = control.throttle

            # Extract specific fields from the odometry message
            odometry_pose_x = odometry.pose.pose.position.x
            odometry_pose_y = odometry.pose.pose.position.y
            odometry_pose_theta = odometry.pose.pose.orientation.z  # Assuming Z is the heading
            odometry_velocity_linear = odometry.twist.twist.linear.x
            odometry_velocity_angular = odometry.twist.twist.angular.z

            # Write the data to CSV
            writer.writerow([control_time, control_steering, control_throttle,
                             odometry_pose_x, odometry_pose_y, odometry_pose_theta,
                             odometry_velocity_linear, odometry_velocity_angular])

    # Shutdown ROS2
    rclpy.shutdown()

if __name__ == "__main__":
    # Argument parsing for trial number
    parser = argparse.ArgumentParser(description="Process ROS2 bag files for a given trial.")
    parser.add_argument('trial', type=str, help="Trial name (e.g., trial_1, trial_2)")
    args = parser.parse_args()

    # Construct file paths based on the trial name and find all playback bag files
    trial_name = args.trial
    playback_bag_pattern = f'{trial_name}/playback_odometry_rosbag_*'
    playback_bag_paths = sorted(glob.glob(playback_bag_pattern))  # List all matching playback bag files

    if not playback_bag_paths:
        print(f"No playback bag files found matching pattern: {playback_bag_pattern}")
        exit(1)

    # Process each playback bag file separately and save its output to a corresponding CSV file
    for bag_path in playback_bag_paths:
        # Get the specific playback file number (e.g., _1, _2)
        bag_name = os.path.basename(bag_path)  # Extract the file name from the path
        csv_output_path = os.path.join(trial_name, f'{bag_name}.csv')  # Construct the corresponding CSV output path

        # Process the current playback bag file
        print(f'Processing {bag_path} and saving to {csv_output_path}')
        process_rosbag(bag_path, csv_output_path)

    # Process the record bag file as before
    record_bag_path = f'{trial_name}/record_rosbag'
    record_csv_path = f'{trial_name}/record_rosbag.csv'
    process_rosbag(record_bag_path, record_csv_path)
