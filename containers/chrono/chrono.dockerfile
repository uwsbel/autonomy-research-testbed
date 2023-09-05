# SPDX-License-Identifier: MIT
ARG CUDA_VERSION
ARG UBUNTU_VERSION
ARG IMAGE_BASE=nvidia/cuda
ARG IMAGE_TAG=${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

FROM ${IMAGE_BASE}:${IMAGE_TAG}

LABEL maintainer="UW Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG PROJECT
ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME="${PROJECT}"
ARG USERHOME="/home/${USERNAME}"
ARG USERSHELL="bash"
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="${USERHOME}/.${USERSHELL}rc"

# Install dependencies
ARG APT_DEPENDENCIES=""
RUN apt-get update && apt-get install --no-install-recommends -y python3-pip sudo ${APT_DEPENDENCIES}

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Install some python packages
ARG PIP_REQUIREMENTS=""
RUN if [ -n "${PIP_REQUIREMENTS}" ]; then \
        pip install ${PIP_REQUIREMENTS}; \
    fi

# Default bash config
RUN mkdir -p ${USERHOME} && touch ${USERSHELLPROFILE}
RUN if [ "${USERSHELL}" = "bash" ]; then \
        echo 'export TERM=xterm-256color' >> ${USERSHELLPROFILE}; \ 
        echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> ${USERSHELLPROFILE}; \
    fi

# Add user and grant sudo permission.
ARG USER_UID=1000
ARG USER_GID=1000
RUN adduser --shell ${USERSHELLPATH} --disabled-password --gecos "" ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}
RUN groupmod -o -g ${USER_GID} ${USERNAME}
RUN usermod -u ${USER_UID} -g ${USER_GID} ${USERNAME}

ARG USER_GROUPS=""
RUN if [ -n "${USER_GROUPS}" ]; then \
			for g in ${USER_GROUPS}; do \
				getent group $g || groupadd $g; \
				usermod -aG $g ${USERNAME}; \
			done; \
    fi

# Move optix file into docker container
ARG OPTIX_SCRIPT
COPY ${OPTIX_SCRIPT} /tmp/optix.sh

# Build chrono install
RUN git clone https://github.com/projectchrono/chrono.git -b main ${USERHOME}/chrono && \
        mkdir ${USERHOME}/chrono/build
RUN chmod +x /tmp/optix.sh && \
        mkdir /opt/optix && \
        /tmp/optix.sh --prefix=/opt/optix --skip-license && \
        rm /tmp/optix.sh
RUN cd ${USERHOME}/chrono/build && cmake ../ -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_BENCHMARKING=OFF \
    -DBUILD_DEMOS=OFF \
    -DBUILD_TESTING=OFF \
    -DENABLE_MODULE_VEHICLE=ON \
    -DENABLE_MODULE_IRRLICHT=ON \
    -DENABLE_MODULE_PYTHON=ON \
    -DENABLE_MODULE_SENSOR=ON \
    -DEigen3_DIR=/usr/lib/cmake/eigen3 \
    -DOptiX_INCLUDE=/opt/optix/include \
    -DOptiX_INSTALL_DIR=/opt/optix \
    -DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
    && ninja && sudo ninja install

# Set user work directory
USER ${USERNAME}
WORKDIR ${USERHOME}
ENV HOME=${USERHOME}
ENV USERSHELLPATH=${USERSHELLPATH}
ENV USERSHELLPROFILE=${USERSHELLPROFILE}
ENV PYTHONPATH="${USERHOME}/chrono/build/bin"

CMD ${USERSHELLPATH}