#!/bin/bash

# This file was adapted from https://github.com/timongentzsch/Jetson_Ubuntu20_Images/blob/master/Dockerfile.base

DEBIAN_FRONTEND=noninteractive

export L4T_RELEASE_MAJOR=34.1
export L4T_RELEASE_MINOR=1
export CUDA=11.4
export SOC="t194"
export L4T_RELEASE=$L4T_RELEASE_MAJOR.$L4T_RELEASE_MINOR

apt-get update && \
    apt-get install -y --no-install-recommends apt-utils software-properties-common wget && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

#
# Jetson debian packages
#
echo $L4T_RELEASE_MAJOR
wget https://repo.download.nvidia.com/jetson/jetson-ota-public.asc -O /etc/apt/trusted.gpg.d/jetson-ota-public.asc
chmod 644 /etc/apt/trusted.gpg.d/jetson-ota-public.asc && \
    apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
    echo "deb https://repo.download.nvidia.com/jetson/common r$L4T_RELEASE_MAJOR main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb https://repo.download.nvidia.com/jetson/${SOC} r$L4T_RELEASE_MAJOR main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# cuda tools setup
apt update && apt install -y libopenblas-dev cuda-tools-11-4

#tensorrt setup
apt update && apt install -y tensorrt python3-libnvinfer-dev 

#torch for cuda 11.4, python3.8
# apt install -y libcurand-dev-11-4 libcufft-dev-11-4
# wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
# wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
pip uninstall -y torch
pip uninstall -y torchvision
# pip install torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
pip install /tmp/scripts/pytorch/torch-1.12.0a0+2c916ef.nv22.3-cp38-cp38-linux_aarch64.whl
pip install /tmp/scripts/pytorch/torchvision-0.13.0-cp38-cp38-linux_aarch64.whl

# pip install torchvision==0.13.0

#
# Tegra setup
#
apt-get update && \
    apt-get install -y libglu1-mesa-dev freeglut3 freeglut3-dev unzip dialog && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf
rm /usr/share/glvnd/egl_vendor.d/50_mesa.json
mkdir -p /usr/share/glvnd/egl_vendor.d/ && \
    echo '{"file_format_version" : "1.0.0" , "ICD" : { "library_path" : "libEGL_nvidia.so.0" }}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
mkdir -p /usr/share/egl/egl_external_platform.d/ && \
    echo '{"file_format_version" : "1.0.0" , "ICD" : { "library_path" : "libnvidia-egl-wayland.so.1" }}' > /usr/share/egl/egl_external_platform.d/nvidia_wayland.json
echo "/usr/local/cuda-$CUDA/targets/aarch64-linux/lib" >> /etc/ld.so.conf.d/nvidia.conf

ldconfig

#
# Update environment
#
export PATH=/usr/local/cuda-$CUDA/bin:/usr/local/cuda/bin:${PATH}
export LD_LIBRARY_PATH=/usr/local/cuda-$CUDA/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/opt/nvidia/vpi1/lib64:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}

export NVIDIA_VISIBLE_DEVICES all
export NVIDIA_DRIVER_CAPABILITIES all
export OPENBLAS_CORETYPE=ARMV8

