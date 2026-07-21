# asr_sdm_video_enhancement_ml

ROS 2 Jazzy C++ node for FiveA+ underwater image enhancement with ONNX Runtime CPU inference.

The model architecture is adapted from UIE_Benckmark's MIT-licensed FiveA+ implementation:
https://github.com/ddz16/UIE_Benckmark

## Dependency Installation
```sh
sudo apt install python3-venv
python3 -m venv venvs/uieb
source venvs/uieb/bin/activate
pip3 install torch torchvision wandb pyiqa pytorch_lightning onnx onnxscript onnxruntime

# Install TensorRT, go to https://developer.nvidia.com/tensorrt/download/11x
# Download specific version you need. For example, TensorRT 11.1.0 GA for Ubuntu 24.04 and CUDA 13.0 to 13.3 DEB local repo Package
sudo dpkg -i ~/Downloads/nv-tensorrt-local-repo-ubuntu2404-11.1.0-cuda-13.3_1.0-1_amd64.deb
```

## Layout

- `src/`, `include/`: C++ ROS 2 node that loads the ONNX model and runs inference.

## Export

The `.pth` to `.onnx` conversion tool is kept outside this ROS package:

```bash
python3 /home/cortin/asr_sdm_robo/src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights /home/cortin/Desktop/FIVE_APLUS_epoch97.pth \
  --output /home/cortin/ros2_ws/src/asr_sdm_video_enhancement_ml/models/five_aplus_epoch97.onnx
```

The checkpoint includes training-only `per_loss.*` weights. The export tool filters those keys before loading the network.

## Build and Run

```bash
# Download onnxruntime form https://github.com/microsoft/onnxruntime/tags, check the version -xxx-xxx
tar -xzvpf onnxruntime-linux-x64-xxx-xxx.tgz
# Copy folder to $HOME/.local/onnxruntime/
# Edit ~/.bashrc, add following
source /opt/ros/jazzy/setup.bash
export ONNXRUNTIME_ROOT="$HOME/.local/onnxruntime/onnxruntime-linux-x64-xxx-xxx"

# Build and run
colcon build --packages-select asr_sdm_video_enhancement_ml
source install/setup.bash
ros2 launch asr_sdm_video_enhancement_ml asr_sdm_video_enhancement_ml.launch.py
```

Defaults:

- subscribes: `/camera/camera/color/image_raw`
- publishes: `/asr_sdm_video_enhancement_ml/image`
- input encodings: `bgr8`, `rgb8`
- minimum input size: `128x128`
