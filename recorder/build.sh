#!/bin/bash
# Laptop: cross-compile vio_rec (C++) for the H15L.
# Env: H15L_SDK (default ~/h15l_sdk_v1.13/sdk)
set -e
cd "$(dirname "$0")"
source "${H15L_SDK:-$HOME/h15l_sdk_v1.13/sdk}/environment-setup-armv8a-poky-linux"
$CXX -std=c++17 -O2 -Wall -Wextra -o vio_rec vio_rec.cpp -lpthread
file vio_rec
