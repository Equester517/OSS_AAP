#!/usr/bin/env bash
# Install/register built lanelet2 libraries so the dynamic loader can find them
# Usage:
#   ./tools/install_lanelet2_libs.sh        # add ld.so.conf.d entry and run ldconfig (recommended)
#   ./tools/install_lanelet2_libs.sh --copy # copy libs to /usr/local/lib and run ldconfig

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_LIB_DIR="$ROOT_DIR/build/install/lib"
CONF_FILE="/etc/ld.so.conf.d/only_missionplanner.conf"

if [ ! -d "$BUILD_LIB_DIR" ]; then
  echo "Error: build lib directory not found: $BUILD_LIB_DIR"
  echo "Please build the project first (e.g. run ./test.sh or cmake && make)."
  exit 1
fi

MODE="conf"
if [ "${1-}" = "--copy" ]; then
  MODE="copy"
fi

if [ "$MODE" = "conf" ]; then
  echo "Registering $BUILD_LIB_DIR in $CONF_FILE (requires sudo)..."
  printf "%s\n" "$BUILD_LIB_DIR" | sudo tee "$CONF_FILE" > /dev/null
  echo "Updating dynamic loader cache (ldconfig)..."
  sudo ldconfig
  echo "Done. The library path $BUILD_LIB_DIR is now registered system-wide via $CONF_FILE."
  echo "You can run your application normally now."
  exit 0
fi

if [ "$MODE" = "copy" ]; then
  echo "Copying libs from $BUILD_LIB_DIR to /usr/local/lib (requires sudo)..."
  sudo cp -v "$BUILD_LIB_DIR"/*.so* /usr/local/lib/
  echo "Updating permissions and dynamic loader cache..."
  sudo ldconfig
  echo "Done. Libraries copied to /usr/local/lib and ldconfig updated."
  exit 0
fi
