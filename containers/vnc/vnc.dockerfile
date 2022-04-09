# SPDX-License-Identifier: MIT
FROM debian:buster

# Install git, supervisor, VNC, & X11 packages
RUN apt-get update && \
	apt-get install -y --no-install-recommends \
			bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb && \
		apt-get clean && \
		rm -rf /var/lib/apt/lists/*

# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768


# noVNC adjustments
# Allows navigation to localhost:8080 instead of localhost:8080/vnc_lite.html
RUN cp /usr/share/novnc/vnc_lite.html /usr/share/novnc/index.html
# Set autoresizing to on
RUN sed -i "/rfb.resizeSession = WebUtil.getConfigVar('resize', false);/a rfb.scaleViewport = true;rfb.resizeSession = true;" /usr/share/novnc/index.html

# Copy over the supervisord.conf file
COPY . /app
CMD ["/app/entrypoint.sh"]

