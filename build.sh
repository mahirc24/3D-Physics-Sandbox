#!/usr/bin/env bash
# Convenience build script. Requires: cmake, a C++17 compiler, and Bullet dev
# libraries (Ubuntu/Debian: `sudo apt-get install libbullet-dev`).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(nproc)"
echo
echo "Built. Try:"
echo "  ./build/sandbox --list"
echo "  ./build/sandbox_tests"
echo "  python3 scripts/run_tests.py"
