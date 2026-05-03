# five_aplus_ros

ROS 2 Jazzy C++ node for FiveA+ underwater image enhancement with ONNX Runtime CPU inference.

The model architecture is adapted from UIE_Benckmark's MIT-licensed FiveA+ implementation:
https://github.com/ddz16/UIE_Benckmark

## Layout

- `cpp_runtime/`: C++ ROS 2 node that loads the ONNX model and runs inference.

## Export

The `.pth` to `.onnx` conversion tool is kept outside this ROS package:

```bash
python3 /home/cortin/asr_sdm_robo/src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights /home/cortin/Desktop/FIVE_APLUS_epoch97.pth \
  --output /home/cortin/ros2_ws/src/five_aplus_ros/models/five_aplus_epoch97.onnx
```

The checkpoint includes training-only `per_loss.*` weights. The export tool filters those keys before loading the network.

## Build

```bash
source /opt/ros/jazzy/setup.bash
export ONNXRUNTIME_ROOT=/home/cortin/.local/onnxruntime/current
cd /home/cortin/ros2_ws
colcon build --packages-select five_aplus_ros
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source /home/cortin/ros2_ws/install/setup.bash
ros2 launch five_aplus_ros five_aplus.launch.py
```

Defaults:

- subscribes: `/camera/camera/color/image_raw`
- publishes: `/five_aplus/image`
- input encodings: `bgr8`, `rgb8`
- minimum input size: `128x128`
