# asr_sdm_video_enhancement_ml

ROS 2 Jazzy C++ node for FiveA+ underwater image enhancement with TensorRT GPU inference via [`tensorrt_common`](../tensorrt_common).

The model architecture is adapted from UIE_Benckmark's MIT-licensed FiveA+ implementation:
https://github.com/ddz16/UIE_Benckmark

## Dependencies

- ROS 2 Jazzy
- [`tensorrt_common`](../tensorrt_common) (CUDA-enabled OpenCV, CUDA Toolkit, TensorRT 11)
- Follow `tensorrt_common` README / `scripts/setup_opencv_cuda_env.sh` before building

## Layout

- `src/`, `include/`: C++ ROS 2 node that builds/loads a TensorRT engine from ONNX and runs inference
- `models/uieb_model.onnx`: default enhancement model

## Export

The `.pth` to `.onnx` conversion tool is kept outside this ROS package:

```bash
python3 /home/cortin/asr_sdm_robo/src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights /home/cortin/Desktop/FIVE_APLUS_epoch97.pth \
  --output /path/to/asr_sdm_video_enhancement_ml/models/uieb_model.onnx
```

The checkpoint includes training-only `per_loss.*` weights. The export tool filters those keys before loading the network.

## Build and Run

```bash
source /opt/ros/jazzy/setup.bash
source src/tensorrt_common/scripts/setup_opencv_cuda_env.sh

colcon build --symlink-install --packages-select tensorrt_common asr_sdm_video_enhancement_ml
source install/setup.bash
ros2 launch asr_sdm_video_enhancement_ml asr_sdm_video_enhancement_ml.launch.py
```

On first run the node builds a TensorRT engine next to the ONNX under `models/engines/` (cached for later launches). Prefer `--symlink-install` so that directory is writable in the source tree.

Defaults:

- model: `share/.../models/uieb_model.onnx`
- subscribes: `/sensing/camera/camera0/image_raw/compressed`
- publishes: `/perception/video_enhancement/image_enhanced/compressed`
- input: `sensor_msgs/CompressedImage` (decoded to BGR)
- output: `sensor_msgs/CompressedImage` (JPEG)
- minimum input size: `128x128`
- frames are letterboxed to the network spatial size before inference; padding is cropped and the result is resized back to the original resolution before publish
