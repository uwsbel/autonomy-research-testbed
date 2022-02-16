FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

LABEL maintainer="UW Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME
ARG USERHOME="/home/$USERNAME"
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="$USERHOME/.${USERSHELL}rc"

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

# Clean up conda
RUN conda clean -a -y

# Default bash config
RUN if [ "$USERSHELL" = "bash" ]; then \
			echo 'export TERM=xterm-256color' >> $USERSHELLPROFILE; \ 
			echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> $USERSHELLPROFILE; \
		fi

# Set user work directory
WORKDIR $USERHOME
ENV HOME=$USERHOME
ENV USERSHELLPATH=$USERSHELLPATH

COPY ./entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]
CMD $USERSHELLPATH
