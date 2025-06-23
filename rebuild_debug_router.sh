#!/bin/bash

# Script to rebuild the router project with debug information
# This is a workaround for the CMake issues

set -e

echo "Cleaning build directory..."
rm -rf build/router
mkdir -p build/router/obj

echo "Setting up build environment..."
cd build/router

# Set debug flags manually
export CMAKE_C_FLAGS="-O0 -g -DDEBUG"
export CMAKE_CXX_FLAGS="-O0 -g -DDEBUG"

echo "Running CMake with debug flags..."
cmake -DCMAKE_BUILD_TYPE=Debug -DUSE_ANCHOR=OFF ../.. || {
    echo "CMake failed, trying alternative approach..."
    # Alternative: manually set compiler flags in the toolchain
    export CFLAGS="-O0 -g -DDEBUG"
    export CXXFLAGS="-O0 -g -DDEBUG"
    cmake -DUSE_ANCHOR=OFF ../..
}

echo "Building project..."
make -j$(nproc)

echo "Build complete!"
echo "Debug information should now be available in build/router/router" 