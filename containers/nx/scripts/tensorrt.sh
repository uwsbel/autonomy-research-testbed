#!/bin/bash

# This file was adapted from https://gist.github.com/yury-sannikov/d5bc8f7c8004ae6d1eb81a8d74cbaa6a

DEBIAN_FRONTEND=noninteractive

apt-get update && \
	apt-get install -yqq git python3.8 python3.8-dev python3-pip build-essential curl wget cmake

EXT_PATH=~/external && \
	mkdir -p $EXT_PATH && cd $EXT_PATH && \
	git clone https://github.com/pybind/pybind11.git 

PYTHON_VERSION=`python -c 'import platform;print(platform.python_version())'` && \
	EXT_PATH=~/external && \
	mkdir Python-${PYTHON_VERSION} && wget -qO- https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz | tar xvz --strip-components=1 -C Python-${PYTHON_VERSION} && \
	mkdir -p ${EXT_PATH}/python3.8/include && mv Python-${PYTHON_VERSION}/Include/* ${EXT_PATH}/python3.8/include/ && rm -rf Python-${PYTHON_VERSION} && \
	cp /usr/include/aarch64-linux-gnu/python3.8/pyconfig.h ~/external/python3.8/include/

mkdir /workspace && cd /workspace && git clone https://github.com/NVIDIA/TensorRT.git && \
	mkdir -p /workspace/TensorRT/python/include/onnx

# monkeypatch the environment
cd /workspace/TensorRT/python && \
	echo "cp -r /usr/local/cuda-10.2/targets/aarch64-linux/include/* ./include/\n"\
	"cp -r /usr/include/aarch64-linux-gnu/* ./include/onnx/\n"\
	"EXT_PATH=/root/external PYTHON_MAJOR_VERSION=3 PYTHON_MINOR_VERSION=8 TARGET_ARCHITECTURE=aarch64 ./build.sh\n"\
	"cp ./build/dist/* /wheel\n" >> ./bootstrap.sh && \
	chmod +x ./bootstrap.sh

sed '$ i\/bin/bash /workspace/TensorRT/python/bootstrap.sh' /ros_entrypoint.sh
