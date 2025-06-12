#!/bin/bash
set -e

# Script to download and build local copies of raylib and bullet
# Libraries will be installed under external/

EXT_DIR="$(dirname "$0")/../external"
mkdir -p "$EXT_DIR"
cd "$EXT_DIR"

# Fetch raylib
if [ ! -d raylib ]; then
  git clone --depth=1 https://github.com/raysan5/raylib.git
fi

mkdir -p raylib/build && cd raylib/build
cmake .. -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX="$EXT_DIR/raylib_install"
make -j$(nproc)
make install
cd "$EXT_DIR"

# Fetch bullet
if [ ! -d bullet3 ]; then
  git clone --depth=1 https://github.com/bulletphysics/bullet3.git
fi
mkdir -p bullet3/build && cd bullet3/build
cmake .. -DBUILD_SHARED_LIBS=OFF -DCMAKE_INSTALL_PREFIX="$EXT_DIR/bullet_install"
make -j$(nproc)
make install

echo "Dependencies installed to $EXT_DIR"
