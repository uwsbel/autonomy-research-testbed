# SPDX-License-Identifier: MIT
# This snippet runs rosdep install on the workspace
# NOTE: ROS_DISTRO is a required ARG and ROS must be installed prior to calling this snippet

# Install rosdep
RUN apt-get update && \
      apt-get install -y python3-rosdep && \
      (if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi) && \
      rosdep update && \
      apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

ARG ROS_DISTRO
ARG ROS_WORKSPACE="./workspace"
ARG ROSDEP_METAPACKAGE=""
ARG ROS_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}/"
COPY ${ROS_WORKSPACE}/src /tmp/workspace/src
RUN . ${ROS_INSTALL_PREFIX}/setup.sh && \
    cd /tmp/workspace && \
    apt-get update && \
    if [ -z "$ROSDEP_METAPACKAGE" ]; then \
        ROSDEP_FROM_PATHS="src"; \
    else \
        ROSDEP_FROM_PATHS=$(colcon list --packages-up-to $ROSDEP_METAPACKAGE | awk '{print $2}' | tr '\n' ' '); \
    fi && \
    rosdep install --from-paths $ROSDEP_FROM_PATHS --ignore-src -r -y && \
    rm -rf /tmp/workspace && \
    apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*
