# syntax = devthefuture/dockerfile-x
# SPDX-License-Identifier: MIT
# This snippet helps with setup on agx specific systems

# Add Isaac apt repositories
# Without this, humble packages on focal (the default ubuntu version for AGX) can't be found
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    grep -qxF 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' /etc/apt/sources.list || \
    echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | tee -a /etc/apt/sources.list
