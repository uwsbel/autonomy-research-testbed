#!/bin/bash

# This file was adapted from https://github.com/timongentzsch/Jetson_Ubuntu20_Images/blob/master/Dockerfile.pytorch

DEBIAN_FRONTEND=noninteractive

# 
# install pytorch deps
#
apt-get update && \
    apt-get install -y --no-install-recommends python3-pip python3-dev libopenblas-dev libjpeg-dev zlib1g-dev sox libsox-dev libsox-fmt-all unzip && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# 
# install prebuild pytorch binaries
#
cd /tmp/scripts/pytorch
 unzip torch-1.9.0-python3.8-aarch64.zip && \
     pip3 install torch-1.9.0-cp38-cp38-linux_aarch64.whl \
         torchvision-0.10.0a0+300a8a4-cp38-cp38-linux_aarch64.whl \
         torchaudio-0.10.0a0+ee74056-cp38-cp38-linux_aarch64.whl

# 
# install numpy via pip
#
pip3 install numpy --upgrade

export PATH=/usr/local/cuda/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/usr/local/cuda/targets/aarch64-linux/lib:$LD_LIBRARY_PATH
