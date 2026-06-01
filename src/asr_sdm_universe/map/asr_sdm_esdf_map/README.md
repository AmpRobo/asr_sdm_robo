# asr_sdm_esdf_map

Incremental ESDF map package for ROS 2.

## Launch

```sh
ros2 launch asr_sdm_esdf_map esdf_map_launch.py
```

The input mode is selected in `config/esdf_map_config.yaml`:

```yaml
esdf_map.input_mode: "vio"    # VIO sparse points + VIO pose
# esdf_map.input_mode: "depth" # depth image + VIO pose
```

`vio` and `depth` are mutually exclusive. The node only subscribes to the topics needed by the selected mode.

## Input modes

### `vio`

Uses sparse VIO map points and the VIO pose:

```yaml
esdf_map.vio_pose_topic: "/localization/video_inertial_odom/pose"
esdf_map.vio_points_topic: "/localization/video_inertial_odom/points"
esdf_map.vio_points_ns_filter: "pts"
esdf_map.vio_points_accumulate: true
```

### `depth`

Uses the RealSense depth image and the existing VIO pose. No odometry topic is required:

```yaml
esdf_map.depth_topic: "/sensing/camera/realsense/depth"
esdf_map.pose_topic: "/localization/video_inertial_odom/pose"
```

`/localization/video_inertial_odom/pose` is expected to be `geometry_msgs/msg/PoseWithCovarianceStamped`.
The VIO sparse `/points` topic is not required in `depth` mode.

## Output topics

```text
/esdf_map/cloud
/esdf_map/occupancy_inflate
/esdf_map/esdf
/esdf_map/occupied_map
/esdf_map/esdf_3d
/esdf_map/esdf_distance
/esdf_map/update_range
```

`/esdf_map/esdf_3d` is kept as the RViz visualization topic. Its `intensity` is normalized from `abs(signed_distance)`, so it is not intended as the planner or binary-map source.

`/esdf_map/esdf_distance` is published as a `sensor_msgs/msg/PointCloud2` using `pcl::PointXYZI`. The fields are `x y z intensity`, where `intensity` is the real signed ESDF distance in meters from `distance_buffer_all_`, without normalization and without `abs()`. Use this topic when saving `esdf.bin` for planning.

`/esdf_map/occupied_map` is published as a `visualization_msgs/msg/Marker` with `TRIANGLE_LIST` mesh faces. The occupied cells are rendered as a cube-shaped surface mesh, so RViz2 shows square/cube occupied blocks while still using mesh triangles instead of `CUBE_LIST`. The marker alpha is set explicitly for RViz2, and `demo.rviz` enables the `occupied_map` marker namespace.


Occupied map mesh parameters:

```yaml
esdf_map.occupied_map_mesh_resolution: 0.30
esdf_map.occupied_map_mesh_max_height_gap: 0.60
```
