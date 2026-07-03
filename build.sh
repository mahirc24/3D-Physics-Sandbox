#!/usr/bin/env bash
# Convenience build script. Requires: cmake, a C++17 compiler, and Bullet dev
# libraries (Ubuntu/Debian: `sudo apt-get install libbullet-dev`).
# For the real-time viewer, also install GLFW:
#   macOS:  brew install glfw
#   Ubuntu: sudo apt-get install libglfw3-dev libgl-dev
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(sysctl -n hw.ncpu 2>/dev/null || nproc)"
echo
echo "Built. Try:"
echo "  ./build/sandbox --list"
echo "  ./build/sandbox_tests"
echo "  python3 scripts/run_tests.py"
echo "  ./build/viewer --scenario stack     # real-time 3D window (needs GLFW)"
