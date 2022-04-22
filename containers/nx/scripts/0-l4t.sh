#!/bin/bash

# This file was adapted from https://github.com/timongentzsch/Jetson_Ubuntu20_Images/blob/master/Dockerfile.base DEBIAN_FRONTEND=noninteractive

L4T_RELEASE_MAJOR=32.6
L4T_RELEASE_MINOR=1
CUDA=10.2
SOC="t194"

L4T_RELEASE=$L4T_RELEASE_MAJOR.$L4T_RELEASE_MINOR 
apt-get update && \
    apt-get install -y --no-install-recommends wget apt-utils software-properties-common && \
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
    echo "deb https://repo.download.nvidia.com/jetson/common r$L4T_RELEASE_MAJOR main" > /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    echo "deb https://repo.download.nvidia.com/jetson/${SOC} r$L4T_RELEASE_MAJOR main" >> /etc/apt/sources.list.d/nvidia-l4t-apt-source.list && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

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
# Update gcc version since we're using CUDA 10.2
# 

apt update && apt install gcc-8 g++-8 -y
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 80 --slave /usr/bin/g++ g++ /usr/bin/g++-8

#
# Update environment
#
echo "export PATH=/usr/local/cuda-$CUDA/bin:/usr/local/cuda/bin:\$PATH" >> $USERSHELLPROFILE
echo "export LD_LIBRARY_PATH=/usr/local/cuda-$CUDA/targets/aarch64-linux/lib:\$LD_LIBRARY_PATH" >> $USERSHELLPROFILE
echo "export LD_LIBRARY_PATH=/opt/nvidia/vpi1/lib64:\$LD_LIBRARY_PATH" >> $USERSHELLPROFILE
echo "export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra:\$LD_LIBRARY_PATH" >> $USERSHELLPROFILE
echo "export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/tegra-egl:\$LD_LIBRARY_PATH" >> $USERSHELLPROFILE

echo "export NVIDIA_VISIBLE_DEVICES=all" >> $USERSHELLPROFILE
echo "export NVIDIA_DRIVER_CAPABILITIES=all" >> $USERSHELLPROFILE
echo "export OPENBLAS_CORETYPE=ARMV8" >> $USERSHELLPROFILE

