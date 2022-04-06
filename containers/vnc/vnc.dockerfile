# SPDX-License-Identifier: MIT
FROM theasp/novnc:latest 

# Make a few edits to the theasp/novnc container to function for our needs

# VNC specific adjustments so you can use it outside of VNC
ARG VNC_PASSWORD
RUN [ -z "$VNC_PASSWORD" ] || sed -i "s/x11vnc -forever -shared/x11vnc -forever -shared -passwd $VNC_PASSWORD/g" /app/conf.d/x11vnc.conf

# noVNC adjustments
# Allows navigation to localhost:8080 instead of localhost:8080/vnc_lite.html
RUN cp /usr/share/novnc/vnc_lite.html /usr/share/novnc/index.html
# Set autoresizing to on
RUN sed -i "/rfb.resizeSession = WebUtil.getConfigVar('resize', false);/a rfb.scaleViewport = true;rfb.resizeSession = true;" /usr/share/novnc/index.html
