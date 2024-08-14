# Install ACADOS

# Set environment variables for ACADOS
ENV ACADOS_INSTALL_DIR /opt/acados
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
    && rm -rf /var/lib/apt/lists/*
    
# Clone the ACADOS repository
RUN git clone https://github.com/acados/acados.git $ACADOS_INSTALL_DIR

# Build and install ACADOS
WORKDIR $ACADOS_INSTALL_DIR
RUN git submodule update --init --recursive \
    && mkdir -p build \
    && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install

# Install the Python interface for ACADOS
WORKDIR $ACADOS_INSTALL_DIR/interfaces/acados_template
RUN pip3 install --no-cache-dir -e .

# Set environment variables for using ACADOS Python interface
ENV PYTHONPATH $ACADOS_INSTALL_DIR/interfaces/acados_template:$PYTHONPATH
