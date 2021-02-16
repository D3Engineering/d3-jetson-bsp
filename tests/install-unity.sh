#!/bin/bash

set -xEeuo pipefail

# Other dependencies
sudo apt install -y linux-libc-dev	# /usr/include/linux

# Retrieve
sudo apt install -y python3-pip ninja-build
pip3 install meson
git clone https://github.com/ThrowTheSwitch/Unity.git
cd Unity

# Build
mkdir build
cd build
meson ..
ninja
cd ..

# Install
sudo mkdir -p /usr/local/include/UnityTest
sudo install -v -m 0644 src/unity.h /usr/local/include/UnityTest/unity.h
sudo install -v -m 0644 src/unity_internals.h /usr/local/include/UnityTest/unity_internals.h
sudo mkdir -p /usr/local/lib/UnityTest
sudo install -v build/meson-out/libunity.a /usr/local/lib/UnityTest/libunity.a
