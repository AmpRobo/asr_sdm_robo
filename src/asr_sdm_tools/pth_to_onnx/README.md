# pth_to_onnx

FiveA+ `.pth` to `.onnx` conversion tool.

```bash
python3 /home/cortin/asr_sdm_robo/src/asr_sdm_tools/pth_to_onnx/export_five_aplus_onnx.py \
  --weights /home/cortin/Desktop/FIVE_APLUS_epoch97.pth \
  --output /home/cortin/ros2_ws/src/five_aplus_ros/models/five_aplus_epoch97.onnx
```

The ROS 2 package uses the generated ONNX model at runtime; this exporter is
kept outside the ROS package.
