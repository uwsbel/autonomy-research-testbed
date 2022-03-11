mkdir -p ./build
cd ./build
CI_PROJECT_DIR=/root/chrono
echo $CI_PROJECT_DIR
export NP_INCL=$(python $CI_PROJECT_DIR/contrib/packaging-python/conda/setvarnumpy.py )
MY_PY_VER=3.8

if [ `uname` == Darwin ]; then
    PY_LIB="libpython${MY_PY_VER}.dylib"
else
    PY_LIB="libpython${MY_PY_VER}.so"
fi

export MKL_INTERFACE_LAYER=LP64
export MKL_THREADING_LAYER=INTEL
CONFIGURATION=Release
# Configure step
cmake -DUSE_CCACHE=ON -DCMAKE_INSTALL_PREFIX=$PREFIX \
 -DCMAKE_PREFIX_PATH=$PREFIX \
 -DCMAKE_SYSTEM_PREFIX_PATH=$PREFIX \
 -DCH_INSTALL_PYTHON_PACKAGE=$SP_DIR \
 -DPYTHON_EXECUTABLE:FILEPATH=$PYTHON \
 -DPYTHON_INCLUDE_DIR:PATH=$PREFIX/include/python$MY_PY_VER \
 -DPYTHON_LIBRARY:FILEPATH=$PREFIX/lib/${PY_LIB} \
 -DCMAKE_BUILD_TYPE=$CONFIGURATION \
 -DENABLE_MODULE_IRRLICHT=ON \
 -DENABLE_MODULE_VEHICLE=ON \
 -DENABLE_MODULE_PYTHON=ON \
 -DENABLE_MODULE_SENSOR=ON \
 -DBUILD_DEMOS=OFF \
 -DBUILD_TESTING=OFF \
 -DBUILD_BENCHMARKING=OFF \
 -DBUILD_GMOCK=OFF \
 -DENABLE_MODULE_CASCADE=OFF \
 -DENABLE_MODULE_PARDISO_MKL=ON \
 -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
 -DPYCHRONO_DATA_PATH=../../../../../../share/chrono/data \
 -DOptiX_INSTALL_DIR=/opt/optix-7.2 \
 -DNUMPY_INCLUDE_DIR=$NP_INCL \
 -DUSE_CUDA_NVRTC=OFF \
 -DCUDA_ARCH_NAME=Manual \
 -DCUDA_ARCH_PTX=52 \
 -DCUDA_ARCH_BIN=5.2 \
 ./..
# Build step
# on linux travis, limit the number of concurrent jobs otherwise
# gcc gets out of memory
cmake --build . -j8 --config "$CONFIGURATION"

cmake --build . -j8 --config "$CONFIGURATION" --target install

