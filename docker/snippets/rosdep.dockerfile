# SPDX-License-Identifier: MIT
# This snippet runs rosdep install on the workspace
# NOTE: ROS_DISTRO is a required ARG and ROS must be installed prior to calling this snippet

ARG ROS_DISTRO
ARG ROS_WORKSPACE="./workspace"
COPY ${ROS_WORKSPACE}/src /tmp/workspace/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd /tmp/workspace && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /tmp/workspace && \
    apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*
