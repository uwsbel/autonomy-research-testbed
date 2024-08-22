# Install ACADOS

# Set environment variables for ACADOS
ENV ACADOS_INSTALL_DIR /home/art/acados
ENV BLASFEO_TARGET GENERIC

RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    cmake \
    libblas-dev \
    liblapack-dev \
    libhdf5-dev \
    pkg-config \
    libtool \
    autoconf \
    python3-dev \
    python3-pip \
    python3-setuptools \
    swig \
    libgmp-dev \
    libmpfr-dev \
    libmpc-dev \
    libffi-dev \
    wget \
    curl \
    cargo \
    && rm -rf /var/lib/apt/lists/*

# Clone the ACADOS repository
RUN git clone https://github.com/acados/acados.git $ACADOS_INSTALL_DIR

# Build and install ACADOS
WORKDIR $ACADOS_INSTALL_DIR
RUN git submodule update --init --recursive \
    && mkdir -p build \
    && cd build \
    && cmake .. -DACADOS_WITH_QPOASES=ON \
    && make -j$(nproc) \
    && make install

# Install the Python interface for ACADOS
WORKDIR $ACADOS_INSTALL_DIR/interfaces/acados_template
RUN pip3 install --no-cache-dir -e .

# Set environment variables for using ACADOS Python interface
ENV ACADOS_SOURCE_DIR=$ACADOS_INSTALL_DIR
ENV PYTHONPATH $ACADOS_INSTALL_DIR/interfaces/acados_template:$PYTHONPATH
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib

WORKDIR /home/art
RUN git clone https://github.com/acados/tera_renderer.git \
    && cd tera_renderer \
    && cargo build --release \ 
    && mv target/release/t_renderer $ACADOS_INSTALL_DIR/bin/ \
    && chmod +x $ACADOS_INSTALL_DIR/bin/t_renderer


