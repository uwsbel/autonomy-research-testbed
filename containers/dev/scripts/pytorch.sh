#!/bin/bash

#groupmod -g 987 uucp
#usermod -aG uucp art

#
# install pytorch deps
#
# apt-get update && \
#     apt-get install -y --no-install-recommends python3-pip python3-dev

#
# install pytorch
#
pip3 install --ignore-installed torch torchvision --extra-index-url https://download.pytorch.org/whl/cu113
