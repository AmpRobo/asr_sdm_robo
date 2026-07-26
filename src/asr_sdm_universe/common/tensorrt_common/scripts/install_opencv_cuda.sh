#!/usr/bin/env bash
# Build and install OpenCV with CUDA into a user-local prefix (no sudo required).
#
# Usage:
#   ./scripts/install_opencv_cuda.sh [OPENCV_VERSION] [CUDA_ARCH_BIN]
#
# Defaults:
#   OPENCV_VERSION = 4.14.0  (needed for CUDA 13.x / CCCL)
#   CUDA_ARCH_BIN  = auto from nvidia-smi (fallback 8.6)
#
# Prerequisites:
#   - CUDA_HOME with nvcc
#   - CUDA libs: cudart, cublas, cufft, npp (see CUDA_LIBS_ROOT)
#   - Optional: cuDNN at /usr/include/x86_64-linux-gnu + libcudnn.so
#
# After install:
#   source scripts/setup_opencv_cuda_env.sh
#   colcon build --symlink-install --packages-select tensorrt_common

set -euo pipefail

OPENCV_VERSION="${1:-4.14.0}"
CUDA_ARCH_BIN="${2:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

INSTALL_PREFIX="${OPENCV_CUDA_PREFIX:-${HOME}/.local/opencv-cuda}"
BUILD_ROOT="${OPENCV_CUDA_BUILD_ROOT:-${HOME}/.cache/opencv_cuda_build}"
SRC_ROOT="${BUILD_ROOT}/src"
BUILD_DIR="${BUILD_ROOT}/build-${OPENCV_VERSION}"
CUDA_LIBS_ROOT="${CUDA_LIBS_ROOT:-${HOME}/.local/cuda-libs-13.3/usr/local/cuda-13.3}"

# CUDA toolkit (prefer env, then known user-local path)
if [[ -z "${CUDA_HOME:-}" ]]; then
  if [[ -x "${HOME}/.local/cuda-13.3-root/usr/local/cuda-13.3/bin/nvcc" ]]; then
    export CUDA_HOME="${HOME}/.local/cuda-13.3-root/usr/local/cuda-13.3"
  elif [[ -x /usr/local/cuda/bin/nvcc ]]; then
    export CUDA_HOME=/usr/local/cuda
  fi
fi

if [[ -z "${CUDA_HOME:-}" || ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
  echo "ERROR: CUDA toolkit not found. Set CUDA_HOME to a path containing bin/nvcc."
  exit 1
fi

export PATH="${CUDA_HOME}/bin:${PATH}"

# Combined library search path: toolkit + extracted npp/cublas/cufft packages
CUDA_LIB_DIRS=("${CUDA_HOME}/lib64")
if [[ -d "${CUDA_LIBS_ROOT}/targets/x86_64-linux/lib" ]]; then
  CUDA_LIB_DIRS+=("${CUDA_LIBS_ROOT}/targets/x86_64-linux/lib")
fi
CUDA_LIB_PATH="$(IFS=:; echo "${CUDA_LIB_DIRS[*]}")"
export LD_LIBRARY_PATH="${CUDA_LIB_PATH}:${LD_LIBRARY_PATH:-}"
export LIBRARY_PATH="${CUDA_LIB_PATH}:${LIBRARY_PATH:-}"

find_cuda_lib() {
  local name="$1"
  local p
  for d in "${CUDA_LIB_DIRS[@]}"; do
    for p in "${d}/lib${name}.so" "${d}/lib${name}.so."*; do
      if [[ -e "$p" ]]; then
        # Prefer non-stub real libraries
        if [[ "$p" == *"/stubs/"* ]]; then
          continue
        fi
        readlink -f "$p"
        return 0
      fi
    done
  done
  return 1
}

# Ensure unversioned sonames exist for CMake FindCUDA (symlink beside real libs if needed)
ensure_unversioned() {
  local name="$1"
  local real
  real="$(find_cuda_lib "$name" || true)"
  [[ -n "$real" ]] || return 0
  local dir
  dir="$(dirname "$real")"
  if [[ ! -e "${dir}/lib${name}.so" ]]; then
    ln -sfn "$(basename "$real")" "${dir}/lib${name}.so"
  fi
}

for lib in cublas cublasLt cufft nppc nppial nppicc nppidei nppif nppig nppim nppist nppisu nppitc npps; do
  ensure_unversioned "$lib" || true
done

CUBLAS_LIB="$(find_cuda_lib cublas)"
CUFFT_LIB="$(find_cuda_lib cufft)"
NPPC_LIB="$(find_cuda_lib nppc)"
NPPIAL_LIB="$(find_cuda_lib nppial)"
NPPICC_LIB="$(find_cuda_lib nppicc)"
NPPIDEI_LIB="$(find_cuda_lib nppidei)"
NPPIF_LIB="$(find_cuda_lib nppif)"
NPPIG_LIB="$(find_cuda_lib nppig)"
NPPIM_LIB="$(find_cuda_lib nppim)"
NPPIST_LIB="$(find_cuda_lib nppist)"
NPPISU_LIB="$(find_cuda_lib nppisu)"
NPPITC_LIB="$(find_cuda_lib nppitc)"
NPPS_LIB="$(find_cuda_lib npps)"

missing=()
for pair in \
  "cublas:${CUBLAS_LIB:-}" \
  "cufft:${CUFFT_LIB:-}" \
  "nppc:${NPPC_LIB:-}" \
  "npps:${NPPS_LIB:-}"
do
  name="${pair%%:*}"
  val="${pair#*:}"
  if [[ -z "$val" ]]; then
    missing+=("$name")
  fi
done
if ((${#missing[@]})); then
  echo "ERROR: missing CUDA libraries: ${missing[*]}"
  echo "Extract NVIDIA debs (libnpp-13-3, libcublas-13-3, libcufft-13-3) into:"
  echo "  ${CUDA_LIBS_ROOT}"
  exit 1
fi

if [[ -z "${CUDA_ARCH_BIN}" ]]; then
  CUDA_ARCH_BIN="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ' || true)"
  CUDA_ARCH_BIN="${CUDA_ARCH_BIN:-8.6}"
fi

echo "================================================================================"
echo "OpenCV ${OPENCV_VERSION} + CUDA install"
echo "  CUDA_HOME       = ${CUDA_HOME}"
echo "  CUDA_LIBS_ROOT  = ${CUDA_LIBS_ROOT}"
echo "  CUDA_ARCH_BIN   = ${CUDA_ARCH_BIN}"
echo "  INSTALL_PREFIX  = ${INSTALL_PREFIX}"
echo "  BUILD_DIR       = ${BUILD_DIR}"
echo "  cublas          = ${CUBLAS_LIB}"
echo "  cufft           = ${CUFFT_LIB}"
echo "  nppc            = ${NPPC_LIB}"
echo "================================================================================"

command -v cmake >/dev/null
command -v wget >/dev/null
command -v unzip >/dev/null
command -v ninja >/dev/null 2>&1 && GENERATOR=(-G Ninja) || GENERATOR=()

mkdir -p "${SRC_ROOT}" "${INSTALL_PREFIX}"
cd "${SRC_ROOT}"

if [[ ! -d "opencv-${OPENCV_VERSION}" ]]; then
  echo "Downloading OpenCV ${OPENCV_VERSION}..."
  wget -q --show-progress -O "opencv-${OPENCV_VERSION}.zip" \
    "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.zip"
  unzip -q "opencv-${OPENCV_VERSION}.zip"
fi

if [[ ! -d "opencv_contrib-${OPENCV_VERSION}" ]]; then
  echo "Downloading OpenCV contrib ${OPENCV_VERSION}..."
  wget -q --show-progress -O "opencv_contrib-${OPENCV_VERSION}.zip" \
    "https://github.com/opencv/opencv_contrib/archive/refs/tags/${OPENCV_VERSION}.zip"
  unzip -q "opencv_contrib-${OPENCV_VERSION}.zip"
fi

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Minimal module set needed by tensorrt_common (cuda modules live in contrib)
BUILD_LIST="core,imgproc,imgcodecs,cudev,cudaarithm,cudawarping,cudaimgproc,cudafilters"

cmake "${GENERATOR[@]}" \
  -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -D CMAKE_CXX_STANDARD=17 \
  -D CMAKE_PREFIX_PATH="${CUDA_LIBS_ROOT}/targets/x86_64-linux" \
  -D CMAKE_LIBRARY_PATH="${CUDA_LIB_PATH}" \
  -D OPENCV_EXTRA_MODULES_PATH="${SRC_ROOT}/opencv_contrib-${OPENCV_VERSION}/modules" \
  -D BUILD_LIST="${BUILD_LIST}" \
  -D WITH_CUDA=ON \
  -D WITH_CUDNN=ON \
  -D OPENCV_DNN_CUDA=OFF \
  -D ENABLE_FAST_MATH=ON \
  -D CUDA_FAST_MATH=ON \
  -D WITH_CUBLAS=ON \
  -D WITH_CUFFT=ON \
  -D CUDA_ARCH_BIN="${CUDA_ARCH_BIN}" \
  -D CUDA_ARCH_PTX="" \
  -D CUDA_TOOLKIT_ROOT_DIR="${CUDA_HOME}" \
  -D CUDA_NVCC_FLAGS="-std=c++17" \
  -D CMAKE_CUDA_STANDARD=17 \
  -D CMAKE_CXX_FLAGS="-I${CUDA_LIBS_ROOT}/targets/x86_64-linux/include" \
  -D CUDA_cublas_LIBRARY="${CUBLAS_LIB}" \
  -D CUDA_cufft_LIBRARY="${CUFFT_LIB}" \
  -D CUDA_nppc_LIBRARY="${NPPC_LIB}" \
  -D CUDA_nppial_LIBRARY="${NPPIAL_LIB}" \
  -D CUDA_nppicc_LIBRARY="${NPPICC_LIB}" \
  -D CUDA_nppidei_LIBRARY="${NPPIDEI_LIB}" \
  -D CUDA_nppif_LIBRARY="${NPPIF_LIB}" \
  -D CUDA_nppig_LIBRARY="${NPPIG_LIB}" \
  -D CUDA_nppim_LIBRARY="${NPPIM_LIB}" \
  -D CUDA_nppist_LIBRARY="${NPPIST_LIB}" \
  -D CUDA_nppisu_LIBRARY="${NPPISU_LIB}" \
  -D CUDA_nppitc_LIBRARY="${NPPITC_LIB}" \
  -D CUDA_npps_LIBRARY="${NPPS_LIB}" \
  -D CUDNN_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
  -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so \
  -D WITH_GTK=OFF \
  -D WITH_QT=OFF \
  -D WITH_OPENGL=OFF \
  -D BUILD_EXAMPLES=OFF \
  -D BUILD_TESTS=OFF \
  -D BUILD_PERF_TESTS=OFF \
  -D BUILD_opencv_apps=OFF \
  -D BUILD_opencv_python2=OFF \
  -D BUILD_opencv_python3=OFF \
  -D BUILD_JAVA=OFF \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D OPENCV_ENABLE_NONFREE=OFF \
  "${SRC_ROOT}/opencv-${OPENCV_VERSION}"

echo "Building (this can take a while)..."
if command -v ninja >/dev/null 2>&1; then
  ninja -j"$(nproc)"
  ninja install
else
  make -j"$(nproc)"
  make install
fi

ENV_FILE="${INSTALL_PREFIX}/setup_env.sh"
cat > "${ENV_FILE}" <<EOF
# Generated by install_opencv_cuda.sh
export CUDA_HOME="${CUDA_HOME}"
export PATH="\${CUDA_HOME}/bin:\${PATH}"
export LD_LIBRARY_PATH="${INSTALL_PREFIX}/lib:${CUDA_LIB_PATH}:\${LD_LIBRARY_PATH:-}"
export OpenCV_DIR="${INSTALL_PREFIX}/lib/cmake/opencv4"
export PKG_CONFIG_PATH="${INSTALL_PREFIX}/lib/pkgconfig:\${PKG_CONFIG_PATH:-}"
EOF

cp "${ENV_FILE}" "${SCRIPT_DIR}/setup_opencv_cuda_env.sh"

echo "================================================================================"
echo "Install complete."
ls -1 "${INSTALL_PREFIX}/include/opencv4/opencv2/cudaimgproc.hpp" \
      "${INSTALL_PREFIX}/include/opencv4/opencv2/cudawarping.hpp" \
      "${INSTALL_PREFIX}/include/opencv4/opencv2/cudaarithm.hpp"
echo
echo "Before colcon build:"
echo "  source ${SCRIPT_DIR}/setup_opencv_cuda_env.sh"
echo "  cd <workspace>"
echo "  colcon build --symlink-install --packages-select tensorrt_common"
echo "================================================================================"
