#!/usr/bin/env bash
##
## author: ydx
## description: 升级cmake到3.22.1

set -e

CMAKE_CURRENT_VERSION=`cmake --version | grep version | awk '{print $3}'`

echo "Trying to update cmake"

echo "Downloading cmake-3.22.1.tar.gz"
wget https://cmake.org/files/v3.22/cmake-3.22.1.tar.gz

echo "Unpacking cmake-3.22.1.tar.gz"
tar -zxvf cmake-3.22.1.tar.gz

echo "Installing cmake-3.22.1"
cd cmake-3.22.1
chmod a+x ./configure
sh ./configure

NUM_CORES=`(which nproc > /dev/null 2>&1 && nproc) || sysctl -n hw.logicalcpu || echo 1`
make -j$NUM_CORES

echo "Trying to install cmake-3.22.1 to system directory, you may need to enter your password!"
sudo make install
sudo update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 1 --force

CMAKE_NEW_VERSION=`cmake --version | grep version | awk '{print $3}'`

if [ "$CMAKE_NEW_VERSION" = "$CMAKE_CURRENT_VERSION" ]; then
    echo "Failed to update cmake!"
    exit 1
else
    echo "Successfully update cmake from $CMAKE_CURRENT_VERSION to $CMAKE_NEW_VERSION!"
fi
