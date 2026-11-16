#!/bin/bash
# Laptop: cross-compile Ceres 2.1.0 (an OpenVINS dependency missing from the H15L SDK) as a static library.
#   build_ceres.sh [install prefix (default ~/h15l_sdk_v1.13/ceres_install)]
# Env: H15L_SDK (default ~/h15l_sdk_v1.13/sdk), CERES_TARBALL (local ceres-solver-2.1.0.tar.gz; else downloaded),
#      JOBS (default 4)
set -e
PREFIX=${1:-$HOME/h15l_sdk_v1.13/ceres_install}
H15L_SDK=${H15L_SDK:-$HOME/h15l_sdk_v1.13/sdk}
WORK=$(mktemp -d); trap 'rm -rf "$WORK"' EXIT
TB=${CERES_TARBALL:-$WORK/ceres-solver-2.1.0.tar.gz}
[ -f "$TB" ] || curl -fL -o "$TB" https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.1.0.tar.gz
tar xzf "$TB" -C "$WORK"
. "$H15L_SDK/environment-setup-armv8a-poky-linux"
# static, no gflags/glog (miniglog), Eigen sparse only: nothing extra to ship to the board
cmake -S "$WORK/ceres-solver-2.1.0" -B "$WORK/build" -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF -DMINIGLOG=ON -DGFLAGS=OFF -DEIGENSPARSE=ON -DLAPACK=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF
make -C "$WORK/build" -j"${JOBS:-4}" install
echo "Ceres installed in $PREFIX"
