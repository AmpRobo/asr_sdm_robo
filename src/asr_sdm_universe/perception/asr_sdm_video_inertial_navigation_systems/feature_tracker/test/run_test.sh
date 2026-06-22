#!/usr/bin/env bash
# Build and run the standalone sparse image alignment unit test.
# Does not require ROS -- just OpenCV and Eigen.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

g++ -std=c++14 -O2 -I /usr/include/eigen3 \
    "${PKG_DIR}/src/sparse_img_align.cpp" \
    "${SCRIPT_DIR}/sparse_align_test.cpp" \
    -o /tmp/sparse_align_test \
    $(pkg-config --cflags --libs opencv4)

echo "Built /tmp/sparse_align_test; running:"
/tmp/sparse_align_test
