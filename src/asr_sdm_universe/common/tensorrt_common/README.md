# tensorrt_common

ROS 2 (`ament_cmake`) C++ library for building and running TensorRT engines from ONNX models.

Provides `Engine` for ONNX → TensorRT conversion, engine load/deserialize, and GPU inference with OpenCV CUDA helpers.

## Requirements

| Dependency | Required | Current / tested version | Notes |
|---|---|---|---|
| OS | Yes | **Ubuntu 24.04.4 LTS** (noble) | |
| ROS 2 | Yes | **Jazzy** (`ros-jazzy-ros-base` 0.11.0) | `ament_cmake` 2.5.6 |
| colcon | Yes | **0.3.0** (`python3-colcon-common-extensions`) | |
| CMake | Yes | **3.28.3** | Package declares ≥ 3.8; ≥ 3.17 recommended for `CUDAToolkit` |
| C++ compiler | Yes | **g++ 13.3.0** | C++17 |
| Python | — | **3.12.3** | Used by ROS / optional ModelOpt |
| NVIDIA GPU | Yes | **GeForce RTX 3060** (SM **8.6**) | |
| NVIDIA driver | Yes | **595.71.05** | Driver reports CUDA **13.2** |
| CUDA Toolkit | Yes | **13.3** (`nvcc` V13.3.73) | Plus cublas / cufft / npp (see below) |
| CUDA cuBLAS | Yes* | **13.6.0.2** (`libcublas.so.13`) | *Needed for OpenCV CUDA |
| CUDA NPP | Yes* | **13.1.2.81** (`libnppc.so.13`) | *Needed for OpenCV CUDA |
| cuDNN | Recommended | **9.25.0** (`libcudnn9-cuda-13`) | OpenCV `WITH_CUDNN=ON` |
| TensorRT | Yes | **11.1.0** (`libnvinfer` 11.1.0.106+cuda13.3) | Strongly typed (TRT 11) |
| OpenCV + CUDA | Yes | **4.14.0** (`~/.local/opencv-cuda`) | Not apt 4.6.0 (no CUDA modules) |
| NVIDIA ModelOpt | Optional | *(not installed here)* | Offline FP16 / INT8 for TRT 11 |
| `tensorrt_common` | — | **0.1.0** | This package |

### Tested stack snapshot

Recorded on the development machine used for this package:

```text
OS:              Ubuntu 24.04.4 LTS
ROS 2:           Jazzy (ros-jazzy-ros-base 0.11.0-1noble…, ament_cmake 2.5.6)
colcon:          python3-colcon-common-extensions 0.3.0
CMake:           3.28.3
g++:             13.3.0
Python:          3.12.3

GPU:             NVIDIA GeForce RTX 3060 (compute capability 8.6)
Driver:          595.71.05  (nvidia-smi CUDA Version: 13.2)

CUDA Toolkit:    13.3 (nvcc release 13.3, V13.3.73)
                 CUDA_HOME≈ ~/.local/cuda-13.3-root/usr/local/cuda-13.3
cuBLAS:          13.6.0.2   (from cuda-libs extract / NVIDIA repo)
NPP:             13.1.2.81  (from cuda-libs extract / NVIDIA repo)
cuDNN:           9.25.0     (libcudnn9-cuda-13 / headers 9.25.0)

TensorRT:        11.1.0     (NvInfer 11.1.0, packages 11.1.0.106-1+cuda13.3)
                 trtexec: TensorRT v110100

OpenCV (CUDA):   4.14.0     (~/.local/opencv-cuda; modules: cudaimgproc, cudawarping, …)
OpenCV (apt):    4.6.0      (/usr — CPU only; do not use for this package)

tensorrt_common: 0.1.0
```

Re-check locally:

```bash
nvidia-smi
nvcc --version
python3 - <<'PY'
import re, pathlib
p=pathlib.Path('/usr/include/x86_64-linux-gnu/NvInferVersion.h')
t=p.read_text()
maj=re.search(r'NV_TENSORRT_MAJOR\s+(\d+)', t).group(1)
mn=re.search(r'NV_TENSORRT_MINOR\s+(\d+)', t).group(1)
pt=re.search(r'NV_TENSORRT_PATCH\s+(\d+)', t).group(1)
print(f'TensorRT {maj}.{mn}.{pt}')
PY
pkg-config --modversion opencv4
# or, with CUDA OpenCV env:
# source src/tensorrt_common/scripts/setup_opencv_cuda_env.sh
# grep OpenCV_VERSION $OpenCV_DIR/OpenCVConfig-version.cmake
```

### Environment hints

```bash
# --- CUDA ---
# System install:
export CUDA_HOME=/usr/local/cuda
# Or user-local extract (example):
# export CUDA_HOME=$HOME/.local/cuda-13.3-root/usr/local/cuda-13.3

export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH}"

# Extra CUDA libs if toolkit is incomplete (NPP / cuBLAS extracted separately):
# export LD_LIBRARY_PATH=$HOME/.local/cuda-libs-13.3/usr/local/cuda-13.3/targets/x86_64-linux/lib:$LD_LIBRARY_PATH

# --- TensorRT ---
export TENSORRT_ROOT=/usr                 # Debian/Ubuntu apt packages
# Or tar / custom install:
# export TENSORRT_DIR=/path/to/TensorRT
# export TENSORRT_ROOT=/path/to/TensorRT

# --- OpenCV with CUDA (after scripts/install_opencv_cuda.sh) ---
source src/tensorrt_common/scripts/setup_opencv_cuda_env.sh
# sets OpenCV_DIR, LD_LIBRARY_PATH, CUDA_HOME, PKG_CONFIG_PATH
```

CMake also auto-detects `$HOME/.local/cuda-*-root/usr/local/cuda-*` and `$HOME/.local/opencv-cuda` when env vars are unset.

## Dependency installation

Examples below target **Ubuntu 24.04** + **ROS 2 Jazzy** + **CUDA 13.x** + **TensorRT 11**.  
Always align versions with your driver: `nvidia-smi` (see “CUDA Version” in the top-right).

Quick checklist after everything is installed:

```bash
nvidia-smi
nvcc --version
dpkg -l | grep -E 'libnvinfer|libnvonnx|libcudnn' | head
ls /usr/include/*/NvInfer.h /usr/include/NvInfer.h 2>/dev/null
ls ~/.local/opencv-cuda/include/opencv4/opencv2/cudaimgproc.hpp  # if using the helper script
```

---

### 1. ROS 2 and workspace tools

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep \
  build-essential \
  cmake \
  git \
  ccache \
  wget \
  unzip \
  pkg-config
```

Optional (full desktop): `sudo apt install -y ros-jazzy-desktop`

Initialize `rosdep` once per machine:

```bash
sudo rosdep init   # ignore error if already initialized
rosdep update
```

Enable ROS in every new shell (or add to `~/.bashrc`):

```bash
source /opt/ros/jazzy/setup.bash
```

---

### 2. NVIDIA driver

Install a proprietary NVIDIA driver that supports your GPU, reboot, then verify:

```bash
nvidia-smi
```

You should see the GPU name and a driver-reported **CUDA Version** (e.g. 13.2). The installed CUDA **toolkit** may be equal or older than that number.

---

### 3. CUDA Toolkit

Install a CUDA Toolkit that matches your TensorRT build (e.g. TensorRT built for CUDA 13.3 → install CUDA 13.3 toolkit).

#### 3a. Recommended — NVIDIA CUDA repo (`cuda-keyring`)

```bash
# Download keyring from https://developer.nvidia.com/cuda-downloads
# Example for Ubuntu 24.04:
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update

# Install toolkit (change 13-3 to your series)
sudo apt install -y cuda-toolkit-13-3

# Also ensure runtime math libs used by OpenCV CUDA are present:
sudo apt install -y \
  libcublas-13-3 libcublas-dev-13-3 \
  libcufft-13-3 libcufft-dev-13-3 \
  libnpp-13-3 libnpp-dev-13-3 \
  cuda-cudart-13-3 cuda-cudart-dev-13-3
```

Persist environment:

```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CUDA_HOME=/usr/local/cuda' >> ~/.bashrc
source ~/.bashrc
nvcc --version
```

#### 3b. Ubuntu multiverse package (often older)

```bash
sudo apt install -y nvidia-cuda-toolkit
```

Note: this may not match TensorRT 11 / CUDA 13 and is usually **not** recommended for this package.

#### 3c. Incomplete user-local toolkit (no sudo) — fill missing NPP / cuBLAS

If you only have a partial CUDA tree under e.g. `~/.local/cuda-13.3-root/...` (has `nvcc` + `cudart` but CMake/OpenCV fail on `CUDA_cublas_LIBRARY` / `npp.h`):

```bash
# Download matching debs from NVIDIA CUDA Ubuntu repo, then extract (no sudo):
BASE=https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64
DEST=$HOME/.local/cuda-libs-13.3
mkdir -p /tmp/cuda_pkgs "$DEST" && cd /tmp/cuda_pkgs

# Pin versions to ones listed in that repo's Packages file; examples:
wget -c $BASE/libnpp-13-3_13.1.2.81-1_amd64.deb
wget -c $BASE/libnpp-dev-13-3_13.1.2.81-1_amd64.deb
wget -c $BASE/libcublas-13-3_13.6.0.2-1_amd64.deb
wget -c $BASE/libcufft-13-3_12.3.0.29-1_amd64.deb

for deb in *.deb; do dpkg-deb -x "$deb" "$DEST"; done

export CUDA_HOME=$HOME/.local/cuda-13.3-root/usr/local/cuda-13.3
export CUDA_LIBS_ROOT=$DEST/usr/local/cuda-13.3
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$CUDA_LIBS_ROOT/targets/x86_64-linux/lib:$LD_LIBRARY_PATH
export PATH=$CUDA_HOME/bin:$PATH
```

`scripts/install_opencv_cuda.sh` reads `CUDA_HOME` and optional `CUDA_LIBS_ROOT` (default `~/.local/cuda-libs-13.3/usr/local/cuda-13.3`).

---

### 4. cuDNN

Needed for OpenCV built with `WITH_CUDNN=ON` (this package’s install script enables it).

**Using NVIDIA cuDNN local repo** (after installing the cuDNN local-repo `.deb` from [NVIDIA cuDNN](https://developer.nvidia.com/cudnn)):

```bash
sudo apt update
sudo apt install -y \
  libcudnn9-cuda-13 \
  libcudnn9-dev-cuda-13 \
  libcudnn9-headers-cuda-13
```

Verify:

```bash
ls /usr/include/x86_64-linux-gnu/cudnn.h /usr/include/cudnn.h 2>/dev/null
ls /usr/lib/x86_64-linux-gnu/libcudnn.so*
```

Point the OpenCV build at these paths if non-default (the helper script already uses `/usr/include/x86_64-linux-gnu` + `libcudnn.so`).

---

### 5. TensorRT

Install TensorRT **development** packages matching your CUDA major version (`NvInfer.h`, `libnvinfer`, `libnvonnxparser`, `libnvinfer_plugin`).

#### 5a. NVIDIA TensorRT local apt repo (recommended)

After installing the TensorRT local-repo package (e.g. `nv-tensorrt-local-repo-ubuntu2404-*-cuda-13.3_*.deb` from NVIDIA):

```bash
sudo apt update
sudo apt install -y \
  libnvinfer-dev \
  libnvinfer-plugin-dev \
  libnvonnxparsers-dev \
  libnvinfer-headers-dev \
  libnvinfer-bin
```

`libnvinfer-bin` provides `trtexec`.

Verify:

```bash
dpkg -l | grep -E 'libnvinfer|libnvonnx'
ls /usr/include/x86_64-linux-gnu/NvInfer.h /usr/include/NvInfer.h 2>/dev/null
which trtexec
trtexec --help | head
```

#### 5b. Tar / custom prefix

```bash
# After extracting the TensorRT tarball:
export TENSORRT_ROOT=/path/to/TensorRT
export LD_LIBRARY_PATH=$TENSORRT_ROOT/lib:$LD_LIBRARY_PATH
```

#### 5c. Extract debs without sudo (optional)

```bash
DEST=$HOME/libs/TensorRT
mkdir -p "$DEST"
for deb in /var/nv-tensorrt-local-repo-*/libnvinfer*.deb \
           /var/nv-tensorrt-local-repo-*/libnvonnxparsers*.deb; do
  dpkg-deb -x "$deb" "$DEST"
done
export TENSORRT_ROOT=$DEST/usr
export LD_LIBRARY_PATH=$DEST/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
# Headers live under $DEST/usr/include/x86_64-linux-gnu
```

This package’s CMake searches `include/` and `include/x86_64-linux-gnu/` under `TENSORRT_ROOT` (default `/usr`).

---

### 6. OpenCV with CUDA

Stock Ubuntu `libopencv-dev` **does not** ship CUDA modules. Build a CUDA-enabled OpenCV.

#### 6a. Helper script (user-local, no sudo) — recommended for this repo

Prerequisites: working `CUDA_HOME`/`nvcc`, cuBLAS/NPP/cuFFT libs, and preferably cuDNN.

```bash
cd src/tensorrt_common

# CUDA 13.x needs OpenCV >= 4.14.0 (script default).
# Second arg = GPU compute capability (https://developer.nvidia.com/cuda-gpus), e.g. RTX 3060 → 8.6
./scripts/install_opencv_cuda.sh          # 4.14.0 + auto-detect arch
# or:
./scripts/install_opencv_cuda.sh 4.14.0 8.6

source ./scripts/setup_opencv_cuda_env.sh
```

Install prefix: `~/.local/opencv-cuda`  
Modules built: `core`, `imgproc`, `imgcodecs`, `cudev`, `cudaarithm`, `cudawarping`, `cudaimgproc`, `cudafilters`.

Confirm:

```bash
ls ~/.local/opencv-cuda/include/opencv4/opencv2/cudaimgproc.hpp \
   ~/.local/opencv-cuda/include/opencv4/opencv2/cudawarping.hpp \
   ~/.local/opencv-cuda/include/opencv4/opencv2/cudaarithm.hpp \
   ~/.local/opencv-cuda/lib/libopencv_cudaimgproc.so
```

Optional apt packages that help the OpenCV build (image codecs / video):

```bash
sudo apt install -y \
  libavcodec-dev libavformat-dev libswscale-dev \
  libjpeg-dev libpng-dev libtiff-dev \
  libtbb-dev
```

#### 6b. System-wide OpenCV + CUDA (requires sudo)

Use the same CMake flags as the script, but set `CMAKE_INSTALL_PREFIX=/usr/local` and run `sudo make install && sudo ldconfig`. See `scripts/install_opencv_cuda.sh` for the full flag list.

---

### 7. NVIDIA ModelOpt (optional, TensorRT 11 FP16 / INT8)

TensorRT 11 is strongly typed: it will **not** auto-cast FP32 ONNX via `BuilderFlag::kFP16`. For FP16/INT8 engines, preprocess ONNX offline:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install nvidia-modelopt[onnx]

# FP16 / mixed precision
python -m modelopt.onnx.autocast --onnx_path model.onnx

# INT8 quantization (needs calibration data)
python -m modelopt.onnx.quantization \
  --onnx_path model.onnx \
  --calibration_data data.npz
```

Then point `Engine::build` / `trtexec` at the converted ONNX. Do **not** use `Precision::INT8` in this package (it throws by design).

---

### 8. Workspace `rosdep` (optional)

From the colcon workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

`rosdep` only covers keys declared in `package.xml` (e.g. `libopencv-dev`).  
**CUDA Toolkit, cuDNN, TensorRT, and CUDA-enabled OpenCV must still be installed as in sections 3–6.**

---

### 9. One-shot environment before `colcon build`

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/ws/src/tensorrt_common/scripts/setup_opencv_cuda_env.sh
# ensure TensorRT libs are visible if not on default paths:
# export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

cd /path/to/ws
colcon build --symlink-install --packages-select tensorrt_common
source install/setup.bash
```

## Package layout

```
tensorrt_common/
├── CMakeLists.txt
├── package.xml
├── include/tensorrt_common/engine.hpp
├── src/engine.cpp
├── test/tensorrt_common_test.cpp
├── scripts/
│   ├── install_opencv_cuda.sh      # build OpenCV+CUDA into ~/.local/opencv-cuda
│   └── setup_opencv_cuda_env.sh    # export OpenCV_DIR / CUDA / LD_LIBRARY_PATH
└── model/                          # example UIE ONNX / engine
    ├── uie_model.onnx
    ├── FIVE_APLUS_FWHT.onnx.data   # external weights for the ONNX
    └── uie_model.engine            # optional prebuilt engine
```

## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source src/tensorrt_common/scripts/setup_opencv_cuda_env.sh   # if using the helper OpenCV install
cd /path/to/test_ws
colcon build --symlink-install --packages-select tensorrt_common
source install/setup.bash
```

If CUDA, TensorRT, or OpenCV CUDA is missing, CMake prints a warning and skips building the library.

Disable the test executable:

```bash
colcon build --packages-select tensorrt_common \
  --cmake-args -DTENSORRT_COMMON_BUILD_BENCHMARK=OFF
```

## Use as a dependency

**package.xml**

```xml
<depend>tensorrt_common</depend>
```

**CMakeLists.txt**

```cmake
find_package(tensorrt_common REQUIRED)
target_link_libraries(your_target PUBLIC tensorrt_common::tensorrt_common)
```

**C++**

```cpp
#include "tensorrt_common/engine.hpp"

Options options;
options.precision = Precision::FP16;
options.optBatchSize = 1;
options.maxBatchSize = 1;

Engine engine(options);
engine.build("path/to/model.onnx");   // writes under <onnx_dir>/engines/
engine.loadNetwork("path/to/model.onnx");

// inputs: [input][batch][cv::cuda::GpuMat]
std::vector<std::vector<cv::cuda::GpuMat>> inputs;
std::vector<std::vector<std::vector<float>>> featureVectors;
engine.runInference(inputs, featureVectors);
```

`Engine::build` only regenerates the engine when the matching file is not already present under `<onnx_dir>/engines/`. The filename embeds GPU name, precision tag, and batch options.

### TensorRT 11 notes

TensorRT 11 uses **strongly typed** networks:

- `BuilderFlag::kFP16` / `kINT8` and INT8 calibrators are **removed**.
- Pass `createNetworkV2(0)` (explicit batch + strong typing are always on).
- For FP16 performance, convert ONNX offline first:

```bash
python -m modelopt.onnx.autocast --onnx_path model.onnx
```

- For INT8, quantize with ModelOpt then build the quantized ONNX (do not use `Precision::INT8` in this package — it will throw).
- Inference uses name-based tensor APIs (`setTensorAddress`, `enqueueV3`, `getNbIOTensors`).

## Benchmark / convert with the test binary

```bash
ros2 run tensorrt_common tensorrt_common_test /path/to/model.onnx
# or:
./install/tensorrt_common/lib/tensorrt_common/tensorrt_common_test /path/to/model.onnx
```

This builds (if needed), loads the engine, runs a warmup, then benchmarks inference. Supply an input image path that exists for your model (default in the test source expects `../inputs/team.jpg`).

### Convert ONNX → engine with `trtexec` (alternative)

If the package is not built yet, TensorRT’s `trtexec` can produce an engine directly:

```bash
# Ensure external ONNX weight files sit next to the .onnx
trtexec \
  --onnx=src/tensorrt_common/model/uie_model.onnx \
  --saveEngine=src/tensorrt_common/model/uie_model.engine \
  --memPoolSize=workspace:1024M
```

Example UIE model I/O: input / output `[1, 3, 256, 256]`.

## License

MIT — see [LICENSE](LICENSE).
