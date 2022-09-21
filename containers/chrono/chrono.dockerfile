FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

LABEL maintainer="UW Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME
ARG USERHOME="/home/$USERNAME"
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="$USERHOME/.${USERSHELL}rc"
ARG PYTHONINSTALLPATH="/home/art/chrono/build/bin"

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

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    	/bin/bash /tmp/miniconda.sh -b -p /opt/conda && \
			rm -rf /tmp/miniconda.sh	

# Put conda in path so we can use conda activate
ENV PATH=$CONDA_DIR/bin:$PATH

# Install chrono's packages
ARG PIP_DEPENDENCIES
ARG CONDA_DEPENDENCIES
ARG CONDA_CHANNELS
RUN if [ -n "$CONDA_DEPENDENCIES" ]; then \
			for c in $CONDA_CHANNELS; do \
				conda config --append channels $c;	\
			done; \
			unset CONDA_CHANNELS; \
			conda install $CONDA_DEPENDENCIES; \
    fi
RUN if [ -n "$PIP_DEPENDENCIES" ]; then \
      pip install $PIP_DEPENDENCIES; \
    fi

# Clean up conda
RUN conda clean -a -y

# Run any user scripts
# Should be used to install additional packages or customize the shell
# Files in scripts should be executable (chmod +x) or else they may not run
# note I added this from the file below
#ARG SCRIPTS_DIR="containers/${CONTAINERNAME}/scripts"
#COPY $SCRIPTS_DIR tmp/scripts/
#RUN apt-get update && for f in /tmp/scripts/*; do [ -x $f ] && [ -f $f ] && $f || continue; done
# RUN rm -rf /tmp/scripts

# Default bash config
RUN if [ "$USERSHELL" = "bash" ]; then \
			echo 'export TERM=xterm-256color' >> $USERSHELLPROFILE; \ 
			echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> $USERSHELLPROFILE; \
		fi

# Set user work directory
WORKDIR $USERHOME
ENV HOME=$USERHOME
ENV USERSHELLPATH=$USERSHELLPATH
ENV USERSHELLPROFILE=$USERSHELLPROFILE
ENV PYTHONPATH=$PYTHONINSTALLPATH

CMD $USERSHELLPATH

# Build the pychrono install
RUN wget https://uwmadison.box.com/shared/static/97fkm979iuccls990ottx5g5bpva8pwe.sh -O optix75.sh
RUN chmod +x optix75.sh
RUN mkdir /opt/optix75
RUN ./optix75.sh --prefix=/opt/optix75 --skip-license
RUN rm optix75.sh
RUN git clone https://github.com/projectchrono/chrono.git -b feature/sensor
RUN mkdir chrono/build
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
 -DOptiX_INCLUDE=/opt/optix75/include \
 -DOptiX_INSTALL_DIR=/opt/optix75 \
 -DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
 && ninja && sudo ninja install

