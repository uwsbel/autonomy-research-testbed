# SPDX-License-Identifier: MIT
# This snippet install ROS
# NOTE: ROS_DISTRO is a required ARG

# Install ROS
ARG ROS_DISTRO
RUN apt-get update && \
      apt-get install --no-install-recommends curl software-properties-common -y && \
      add-apt-repository universe && \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME}) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
      apt-get update && \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions build-essential && \
      apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Update shell config
RUN echo ". /opt/ros/${ROS_DISTRO}/setup.${USERSHELL}" >> ${USERSHELLPROFILE}
