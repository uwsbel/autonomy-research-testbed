# SPDX-License-Identifier: MIT
# This snippet will install OptiX in /opt/optix
# NOTE: Requires OPTIX_SCRIPT to be set and for there be a file that exists there

# OptiX
ARG OPTIX_SCRIPT
COPY ${OPTIX_SCRIPT} /tmp/optix.sh
RUN chmod +x /tmp/optix.sh && \
    mkdir /opt/optix && \
    /tmp/optix.sh --prefix=/opt/optix --skip-license && \
    rm /tmp/optix.sh
