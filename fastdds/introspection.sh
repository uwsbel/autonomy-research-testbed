#!/bin/bash

configure_fastdds() {
    local mode=$1
    local profile_file="dds_profile_${mode}.xml"
    echo "Configuring FastDDS for ROS2 CLI introspection (${mode^} Mode)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/art/art/fastdds/$profile_file
    ros2 daemon stop
    ros2 daemon start
    echo "Complete."
}

# Prompt user for host or client configuration
read -p "Configure for host or client? (h/c): " choice

if [[ "$choice" == "h" ]]; then
    configure_fastdds "host"
elif [[ "$choice" == "c" ]]; then
    configure_fastdds "client"
else
    echo "Invalid choice. Please enter 'h' for host or 'c' for client."
fi

