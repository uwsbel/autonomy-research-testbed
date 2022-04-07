#!/bin/sh

cd /root/chrono/contrib/package-python/conda

conda config --set anaconda_upload yes
conda build .
