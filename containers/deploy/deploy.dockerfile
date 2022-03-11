
# Modified from sources with the below copyright

# Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

ARG IMAGE
FROM ${IMAGE}

RUN apt update -y && apt install -y vim git gcc g++ libnuma1

RUN echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf

# RUN rm /usr/share/glvnd/egl_vendor.d/50_mesa.json
RUN mkdir -p /usr/share/glvnd/egl_vendor.d/ && echo '\
{\
    "file_format_version" : "1.0.0",\
    "ICD" : {\
        "library_path" : "libEGL_nvidia.so.0"\
    }\
}' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN mkdir -p /usr/share/egl/egl_external_platform.d/ && echo '\
{\
    "file_format_version" : "1.0.0",\
    "ICD" : {\
        "library_path" : "libnvidia-egl-wayland.so.1"\
    }\
}' > /usr/share/egl/egl_external_platform.d/nvidia_wayland.json

RUN echo "/usr/local/cuda-10.0/targets/aarch64-linux/lib" >> /etc/ld.so.conf.d/nvidia.conf

COPY ./dst/bin /usr/local/cuda-10.2/bin
COPY ./dst/nvvm /usr/local/cuda-10.2/nvvm
COPY ./dst/nvvmx /usr/local/cuda-10.2/nvvmx
COPY ./dst/include /usr/local/cuda-10.2/targets/aarch64-linux/include
COPY ./dst/lib64/stubs /usr/local/cuda-10.2/targets/aarch64-linux/lib/stubs
COPY ./dst/lib64/libcudadevrt.a /usr/local/cuda-10.2/targets/aarch64-linux/lib/
COPY ./dst/lib64/libcudart_static.a /usr/local/cuda-10.2/targets/aarch64-linux/lib/

RUN ln -s /usr/local/cuda-10.2 /usr/local/cuda && \
    ln -s /usr/local/cuda-10.2/targets/aarch64-linux/include /usr/local/cuda/include && \
    ln -s /usr/local/cuda-10.2targets/aarch64-linux/lib /usr/local/cuda/lib64

ENV PATH /usr/local/cuda-10.2/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda-10.2/targets/aarch64-linux/lib:${LD_LIBRARY_PATH}

RUN ldconfig

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
