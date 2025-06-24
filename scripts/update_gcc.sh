#!/usr/bin/env bash
##
## author: ydx
## description: update gcc and g++ to gcc-9 g++-9 for C++17 support
## date: 2023-12-25

echo "Trying to update gcc and g++ to gcc-9 g++-9 for C++17 support!"
echo "You may need to enter your password!"

echo "add-apt-repository ppa:ubuntu-toolchain-r/test"
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

echo "update apt-get"
sudo apt-get update

echo "install gcc-9 g++-9, wait a moment!"
sudo apt-get install gcc-9 g++-9

echo "set gcc-9 g++-9 as default gcc g++"
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 20 --slave /usr/bin/g++ g++ /usr/bin/g++-9
