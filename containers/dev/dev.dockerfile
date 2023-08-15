# SPDX-License-Identifier: MIT
ARG ROS_DISTRO=galactic
ARG IMAGE_BASE=ros
ARG IMAGE_TAG=${ROS_DISTRO}

FROM ${IMAGE_BASE}:${IMAGE_TAG}

LABEL maintainer="Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG PROJECT
ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME="${PROJECT}"
ARG USERHOME="/home/${USERNAME}"
ARG USERSHELL="bash"
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="${USERHOME}/.${USERSHELL}rc"

ARG CONTAINERNAME="dev"

# Add user and grant sudo permission.
ARG USER_UID=1000
ARG USER_GID=1000
RUN adduser --shell $USERSHELLPATH --disabled-password --gecos "" $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
RUN groupmod -o -g $USER_GID $USERNAME
RUN usermod -u $USER_UID -g $USER_GID $USERNAME

ARG USER_GROUPS=""
RUN if [ -n "$USER_GROUPS" ]; then \
			for g in $USER_GROUPS; do \
				getent group $g || groupadd $g; \
				usermod -aG $g $USERNAME; \
			done; \
    fi

# Install dependencies
ARG APT_DEPENDENCIES=""
RUN apt-get update && apt-get install --no-install-recommends -y python3-pip $APT_DEPENDENCIES

# Install needed ros packages
ARG ROS_WORKSPACE="workspace"
COPY $ROS_WORKSPACE/src /tmp/workspace/src/
RUN cd /tmp/workspace && apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN rm -rf /tmp/workspace

# Install some python packages
ARG PIP_REQUIREMENTS=""
RUN pip3 install $PIP_REQUIREMENTS

# Run any user scripts
# Should be used to install additional packages or customize the shell
# Files in scripts should be executable (chmod +x) or else they may not run
ARG SCRIPTS_DIR="containers/${CONTAINERNAME}/scripts"
COPY $SCRIPTS_DIR /tmp/scripts/
RUN apt-get update && for f in /tmp/scripts/*; do [ -x $f ] && [ -f $f ] && $f || continue; done
RUN rm -rf /tmp/scripts

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ROS Setup
RUN sed -i 's|source|#source|g' /ros_entrypoint.sh
RUN echo ". /opt/ros/$ROS_DISTRO/setup.sh" >> $USERSHELLPROFILE
RUN echo "[ -f $USERHOME/$PROJECT/workspace/install/setup.$USERSHELL ] && . $USERHOME/$PROJECT/workspace/install/setup.$USERSHELL" >> $USERSHELLPROFILE
RUN /bin/$USERSHELL -c "source /opt/ros/$ROS_DISTRO/setup.$USERSHELL"

# Default bash config
RUN if [ "$USERSHELL" = "bash" ]; then \
			echo 'export TERM=xterm-256color' >> $USERSHELLPROFILE; \ 
			echo 'export PS1="\[\033[38;5;40m\]\h\[$(tput sgr0)\]:\[$(tput sgr0)\]\[\033[38;5;39m\]\w\[$(tput sgr0)\]\\$ \[$(tput sgr0)\]"' >> $USERSHELLPROFILE; \
			echo 'export KERAS_BACKEND="torch"' >> $USERSHELLPROFILE; \ 
		fi

# Set user and work directory
USER $USERNAME
WORKDIR $USERHOME
ENV HOME=$USERHOME
ENV USERSHELLPATH=$USERSHELLPATH
ENV USERSHELLPROFILE=$USERSHELLPROFILE

CMD $USERSHELLPATH
