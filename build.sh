#!/bin/bash
# Script to build all components from scratch, using the maximum available CPU power
#
# Given parameters are passed over to CMake.
# Examples:
#    * ./build_all.sh -DCMAKE_BUILD_TYPE=Debug
#    * ./build_all.sh VERBOSE=1
#
# Written by Tiffany Huang, 12/14/2016
#

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Create build dir
mkdir -p build
cd build

# Configure build
GIT=`which git`
if [ -n "${GIT}" ]
then
    echo "Using git: ${GIT}"
    cmake .. -DGIT_SUBMODULE=ON -DGIT_EXECUTABLE=${GIT} $*
else
    echo "Git not found"
    cmake .. $*
fi

# Determine number of processors
HOST_OS=`uname`
if [ "Darwin" == "${HOST_OS}" ]
then
    CORES=`sysctl -n hw.ncpu`
else
    CORES=`nproc`
fi
echo "Using ${CORES} cores"

# Build
make -j ${CORES}
