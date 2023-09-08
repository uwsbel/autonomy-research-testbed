FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

LABEL maintainer="UW Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME
ARG USERHOME="/home/$USERNAME"
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="$USERHOME/.${USERSHELL}rc"
ARG PYTHONINSTALLPATH="$USERHOME/chrono/build/bin"

ARG CONTAINERNAME="chrono"

# Check for updates
RUN apt update && apt upgrade -y && apt install sudo -y

# Add user and grant sudo permission.
RUN adduser --shell $USERSHELLPATH --disabled-password --gecos "" $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Install some packages
ARG APT_DEPENDENCIES
RUN apt update && apt install -y wget $APT_DEPENDENCIES

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*


USER $USERNAME
RUN sudo chown -R $USERNAME:$USERNAME /opt

# Install chrono's packages
ARG PIP_DEPENDENCIES

RUN if [ -n "$PIP_DEPENDENCIES" ]; then \
      pip install $PIP_DEPENDENCIES; \
    fi


# Default bash config
RUN if [ "$USERSHELL" = "bash" ]; then \
			echo 'export TERM=xterm-256color' >> $USERSHELLPROFILE; \ 
			echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> $USERSHELLPROFILE; \
            echo 'alias python=/usr/bin/python3.8' >> $USERSHELLPROFILE; \
            echo 'alias pip=/usr/bin/pip3' >> $USERSHELLPROFILE; \
		fi

# Set user work directory
WORKDIR $USERHOME
ENV HOME=$USERHOME
ENV USERSHELLPATH=$USERSHELLPATH
ENV USERSHELLPROFILE=$USERSHELLPROFILE
ENV PYTHONPATH=$PYTHONINSTALLPATH

CMD $USERSHELLPATH

# Build the pychrono install
RUN git clone https://github.com/projectchrono/chrono.git -b main
RUN mkdir chrono/build

# Move optix file into docker container
COPY NVIDIA-OptiX-SDK-7.7.0-linux64-x86_64.sh $USERHOME/optix77.sh
RUN sudo chmod +x optix77.sh
RUN mkdir /opt/optix77
RUN $USERHOME/optix77.sh --prefix=/opt/optix77 --skip-license
RUN rm optix77.sh
RUN cd chrono/build && cmake ../ -G Ninja \
 -DCMAKE_BUILD_TYPE=Release \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_POSTPROCESS=ON \
 -DENABLE_MODULE_PYTHON=ON \
 -DENABLE_MODULE_SENSOR=ON \
 -DENABLE_OPENMP=ON \
 -DENABLE_MODULE_VEHICLE=ON \
 -DEigen3_DIR=/usr/lib/cmake/eigen3 \
 -DOptiX_INCLUDE=/opt/optix77/include \
 -DOptiX_INSTALL_DIR=/opt/optix77 \
 -DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
 && ninja && sudo ninja install


