#!/bin/bash

configure_fastdds() {
    local mode=$1
    local profile_file="dds_profile_client.xml"
    echo "Configuring FastDDS for ROS2 CLI introspection)"
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/art/art/tools/$profile_file
    ros2 daemon stop
    ros2 daemon start
    echo "Complete."
}

# Prompt user for host or client configuration
configure_fastdds "client"

