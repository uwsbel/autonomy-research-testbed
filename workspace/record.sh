#!/bin/bash

# Start ROS 2 bag playback in the background
ros2 bag play standStill1/ --remap /fix3_airsim:=/fix &

# Store the PID of the ROS 2 bag playback process
BAG_PID=$!

# Wait for 0.5 seconds to ensure the ROS 2 bag has started
sleep 1.0

# Run your Python script
python3 src/localization/localization_shared_utils/localization_shared_utils/vel_compare_sim.py 

# Optionally, wait for the ROS 2 bag playback process to finish
wait $BAG_PID

# Your script ends here. The Python script should exit on its own when the bag playback stops.
