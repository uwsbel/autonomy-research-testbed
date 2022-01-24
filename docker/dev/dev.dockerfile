ARG ROSDISTRO

FROM ros:${ROSDISTRO}

LABEL maintainer="Simulation Based Engineering Laboratory <negrut@wisc.edu>"

ARG CONTEXT
ARG REPONAME
ARG ROSDISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERSHELL=bash
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="/root/.${USERSHELL}rc"

# Use mirrors instead of main server
RUN sed -i 's|deb http://.*ubuntu.com.* \(focal.*\)|deb mirror://mirrors.ubuntu.com/mirrors.txt \1|g' /etc/apt/sources.list

# Check for updates
RUN apt update && apt upgrade -y

# Install dependencies
COPY ${CONTEXT}/dependencies.txt /tmp/dependencies.txt
RUN apt-get install --no-install-recommends -y `cat /tmp/dependencies.txt`
RUN rm -rf /tmp/dependencies

# Install needed ros packages
COPY workspace/src /tmp/workspace/src/
RUN cd /tmp/workspace && rosdep install --from-paths src --ignore-src -r -y
RUN cd /tmp/ && rm -rf workspace

# Install some python packages
COPY ${CONTEXT}/requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
RUN rm -rf /tmp/requirements.txt

# Run any user scripts
# Should be used to install additional packages
COPY ${CONTEXT}/scripts/ /tmp/scripts/
RUN cd /tmp/scripts && for f in *; do [ -x $f ] && [ -f $f ] && ./$f || exit 0; done
RUN rm -rf /tmp/scripts

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# ROS Setup
RUN sed -i 's|source|#source|g' /ros_entrypoint.sh
RUN echo ". /opt/ros/$ROSDISTRO/setup.sh" >> $USERSHELLPROFILE
RUN echo "[ -f /root/$REPONAME/workspace/install/setup.$USERSHELL ] && . /root/$REPONAME/workspace/install/setup.$USERSHELL" >> $USERSHELLPROFILE
RUN /bin/bash -c "source /opt/ros/$ROSDISTRO/setup.bash"

WORKDIR /root/

ENV USERSHELLPATH=$USERSHELLPATH
CMD $USERSHELLPATH
