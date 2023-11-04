# SPDX-License-Identifier: MIT
# This snippet install Chrono in /opt/chrono
# NOTE: Requires OPTIX_SCRIPT to be set and for there be a file that exists there
# NOTE: ROS needs to be installed, as well

# Install Chrono dependencies
RUN apt-get update && \
        apt-get install --no-install-recommends -y \
        libirrlicht-dev \
        libeigen3-dev \
        git \
        cmake \
        ninja-build \
        python3-colcon-common-extensions \
        swig \
        python3-numpy \
        libxxf86vm-dev \
        freeglut3-dev \
        libglu1-mesa-dev \
        libglfw3-dev \
        libglew-dev \
        wget \
        xorg-dev && \
        apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# OptiX
ARG OPTIX_SCRIPT
COPY ${OPTIX_SCRIPT} /tmp/optix.sh
RUN chmod +x /tmp/optix.sh && \
    mkdir /opt/optix && \
    /tmp/optix.sh --prefix=/opt/optix --skip-license && \
    rm /tmp/optix.sh

# Vulkan
RUN wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | tee /etc/apt/trusted.gpg.d/lunarg.asc && \
    wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list http://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list && \
    apt-get update && apt-get install vulkan-sdk -y && \
    apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Temporarily change to the user so that file permissions are okay
USER ${USERNAME}

# chrono_ros_interfaces
ARG ROS_WORKSPACE_DIR="${USERHOME}/ros_workspace"
ARG CHRONO_ROS_INTERFACES_DIR="${ROS_WORKSPACE_DIR}/src/chrono_ros_interfaces"
RUN mkdir -p ${CHRONO_ROS_INTERFACES_DIR} && \
    git clone https://github.com/AaronYoung5/chrono_ros_interfaces.git ${CHRONO_ROS_INTERFACES_DIR} && \
    cd ${ROS_WORKSPACE_DIR} && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select chrono_ros_interfaces

# Chrono
ARG CHRONO_BRANCH="main"
ARG CHRONO_REPO="https://github.com/projectchrono/chrono.git"
ARG CHRONO_DIR="${USERHOME}/chrono"
ARG CHRONO_INSTALL_DIR="/opt/chrono"
RUN git clone --recursive -b ${CHRONO_BRANCH} ${CHRONO_REPO} ${CHRONO_DIR} && \
    . ${ROS_WORKSPACE_DIR}/install/setup.sh && \
    cd ${CHRONO_DIR}/contrib/build-scripts/vsg/ && \
    sudo bash buildVSG.sh /opt/vsg && \
    cd ${CHRONO_DIR}/contrib/build-scripts/urdf/ && \
    sudo bash buildURDF.sh /opt/urdf && \
    mkdir ${CHRONO_DIR}/build && \
    cd ${CHRONO_DIR}/build && \
    cmake ../ -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_DEMOS=OFF \
        -DBUILD_BENCHMARKING=OFF \
        -DBUILD_TESTING=OFF \
        -DENABLE_MODULE_VEHICLE=ON \
        -DENABLE_MODULE_IRRLICHT=ON \
        -DENABLE_MODULE_PYTHON=ON \
        -DENABLE_MODULE_SENSOR=ON \
        -DENABLE_MODULE_ROS=ON \
        -DENABLE_MODULE_VSG=ON \
        -DENABLE_MODULE_PARSERS=ON \
        -DEigen3_DIR=/usr/lib/cmake/eigen3 \
        -DOptiX_INCLUDE=/opt/optix/include \
        -DOptiX_INSTALL_DIR=/opt/optix \
        -DUSE_CUDA_NVRTC=OFF \
        -DNUMPY_INCLUDE_DIR=$(python3 -c 'import numpy; print(numpy.get_include())') \
        -DCMAKE_INSTALL_PREFIX=${CHRONO_INSTALL_DIR} \
        -Dvsg_DIR=/opt/vsg/lib/cmake/vsg \
        -DvsgImGui_DIR=/opt/vsg/lib/cmake/vsgImGui \
        -DvsgXchange_DIR=/opt/vsg/lib/cmake/vsgXchange \
        -Durdfdom_DIR=/opt/urdf/lib/urdfdom/cmake \
        -Durdfdom_headers_DIR=/opt/urdf/lib/urdfdom_headers/cmake \
        -Dconsole_bridge_DIR=/opt/urdf/lib/console_bridge/cmake \
        -Dtinyxml2_DIR=/opt/urdf/CMake \
        && \
    ninja && sudo sh -c ". ${ROS_WORKSPACE_DIR}/install/setup.sh; ninja install"

# Switch back out of the USER back to root
USER root

# Update shell config
RUN echo ". ${ROS_WORKSPACE_DIR}/install/setup.sh" >> ${USERSHELLPROFILE} && \
    echo "export PYTHONPATH=\$PYTHONPATH:${CHRONO_INSTALL_DIR}/share/chrono/python" >> ${USERSHELLPROFILE} && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${CHRONO_INSTALL_DIR}/lib" >> ${USERSHELLPROFILE}
