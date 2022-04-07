FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

LABEL maintainer="UW Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG DEBIAN_FRONTEND=noninteractive

# Check for updates
RUN apt update && apt upgrade -y

# Install some packages
ARG APT_DEPENDENCIES
RUN apt update && apt install -y wget git build-essential $APT_DEPENDENCIES

# Install miniconda
ENV CONDA_DIR /opt/conda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p /opt/conda

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

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Clone in chrono
RUN git clone https://github.com/projectchrono/chrono.git -b feature/ros_bridge /root/chrono

# Overwrite the conda build script
COPY build.sh /root/chrono/contrib/packaging-python/conda/

# Run the entrypoint
COPY entrypoint.sh /
CMD ["/entrypoint.sh"]
