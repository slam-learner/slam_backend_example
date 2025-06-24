#!/usr/bin/env bash
# install clang and other tools

# set -e

echo "Installing clang and other tools..."
echo "Enter your password when prompted."

sudo apt update
sudo apt install clang-12 clangd-12 llvm-12 llvm-12-dev clang-format-12 clang-tidy-12

sudo update-alternatives --install /usr/bin/clang clang /usr/bin/clang-12 120
sudo update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-12 120
sudo update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-12 120
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-12 120
sudo update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-12 120