# SPDX-License-Identifier: MIT
# This snippet install ROS
# NOTE: ROS_DISTRO is a required ARG

# Install ROS
ARG ROS_DISTRO
ARG ROS_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}/"
RUN apt-get update && \
      apt-get install curl software-properties-common -y && \
      add-apt-repository universe && \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
      apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*
RUN apt-get update && \
      apt-get install -y ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-robot-localization python3-colcon-common-extensions build-essential && \
      apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Update shell config
ARG USERSHELL
ARG USERSHELLPROFILE
RUN echo ". ${ROS_INSTALL_PREFIX}/setup.${USERSHELL}" >> ${USERSHELLPROFILE}
