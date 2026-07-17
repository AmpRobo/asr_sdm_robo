# asr_sdm_esdf_map

ROS 2 library adaptation of Fast-Planner `plan_env` mapping code.

## Core library

The core target is `asr_sdm_esdf_map`. `src/esdf_map.cpp` keeps the Fast-Planner mapping pipeline:

- synchronized depth image + odometry input through `message_filters::ApproximateTime`
- independent world-frame point cloud + odometry input
- probabilistic raycasting into `occupancy_buffer_`
- local clearing and obstacle inflation into `occupancy_buffer_inflate_`
- positive and negative Euclidean distance transforms
- combined signed distance in `distance_buffer_all_`

Default inputs:

```text
/localization/vins/point_cloud       sensor_msgs/msg/PointCloud
/localization/vins/odometry          nav_msgs/msg/Odometry
/sensing/camera/realsense/depth      sensor_msgs/msg/Image
```

The class is used as a library:

```cpp
ESDFMap::Ptr esdf_map(new ESDFMap);
esdf_map->initMap(node);

amprobo::EDTEnvironment::Ptr edt_environment(new amprobo::EDTEnvironment);
edt_environment->setMap(esdf_map);
```

The library does not publish visualization topics. The standalone visualization node is located in `test/`.

At initialization, the library can also load `maps/occupancy.bin` and
`maps/esdf.bin`. Set `esdf_map.preload_map_directory` to an empty string to
disable this data path. Absolute directories are accepted; relative directories
are resolved from the installed package share directory.

### Binary-map resolution conversion

`esdf_map.resolution` is the **target map resolution**. It is used by the
occupancy buffers, ESDF computation, point-cloud voxel centers, and the RViz
`CUBE_LIST` cube edge length. There is no separate visualization resolution.

The current `.bin` files contain world-frame voxel centers but do not store the
source voxel size. Set the independent source-resolution parameter:

```yaml
esdf_map:
  preload_source_resolution: 0.15
```

When the source and target grids align and have the same resolution, records are
loaded directly. Otherwise, each source occupied record is treated as a physical
cube with edge length `preload_source_resolution`. Every target voxel with a
positive-volume AABB intersection is marked occupied. This conservative rule
works for both coarse-to-fine and fine-to-coarse conversion and does not require
an integer resolution ratio.

When conversion is required, the old `esdf.bin` is ignored. The library rebuilds
the complete ESDF from the converted occupancy grid using
`esdf_map.resolution`. The preloaded occupancy map is already treated as
inflated, so no second inflation pass is applied.

## Test

The test launch defaults to preload-only mode, so no sensor topics are required:

```bash
ros2 launch asr_sdm_esdf_map esdf_map_test.launch.py
```

Enable either live input chain explicitly when it is available:

```bash
ros2 launch asr_sdm_esdf_map esdf_map_test.launch.py \
  enable_pointcloud_odom:=true

ros2 launch asr_sdm_esdf_map esdf_map_test.launch.py \
  enable_depth_odom:=true
```

Test-only outputs:

```text
/map/esdf_map/cloud               sensor_msgs/msg/PointCloud2
/map/esdf_map/occupancy_inflate   sensor_msgs/msg/PointCloud2
/map/esdf_map/esdf_distance       sensor_msgs/msg/PointCloud2
/map/esdf_map/esdf                sensor_msgs/msg/PointCloud2
/map/esdf_map/occupied_map        visualization_msgs/msg/Marker
```

`/map/esdf_map/cloud` contains only raw occupied voxels reconstructed from the live
VINS point cloud or synchronized depth-image/odometry input. Preloaded
`occupancy.bin` data is deliberately excluded. `/map/esdf_map/occupancy_inflate`
contains the collision voxels; `occupancy.bin` is treated as already inflated
and is published here without another inflation pass.

`/map/esdf_map/esdf_distance` contains the actual 3D voxel centers and the
unmodified signed ESDF distance in metres in its `intensity` field. It is the
serialization input for `asr_sdm_map_saver`. `/map/esdf_map/esdf` contains the
same voxel centers, but its `intensity` field is normalized to `[0, 1]` for
deterministic RViz coloring. Both clouds and `occupancy_inflate` use the same
`header.stamp` and `header.frame_id` for exact snapshot matching.
`/map/esdf_map/occupied_map` is a
height-colored `visualization_msgs/msg/Marker::CUBE_LIST` built from the same
inflated collision voxels. Each marker point is one occupied voxel center, and
its cube edge length is read directly from `ESDFMap::getResolution()`. Therefore
the target map grid and RViz voxel geometry always use the same
`esdf_map.resolution` parameter. A preloaded `.bin` source resolution is used
only while converting source occupied cubes into that target grid.

The supplied RViz configuration uses the ROS 2
`rviz_default_plugins/PointCloud2` display for `/map/esdf_map/occupancy_inflate`
and `/map/esdf_map/esdf`, with Reliable + Transient Local QoS matching the test
publishers. The ESDF display uses the normalized `intensity` field in `[0, 1]`.
RViz also displays `/localization/vins/odometry` directly with the Odometry plugin.

The only test source is `test/esdf_map_test.cpp`; it links the exported
`asr_sdm_esdf_map` target in the same way as a downstream package.

## Dynamic-object support retained from Fast-Planner

- `src/obj_predictor.cpp` remains part of the shared library, matching Fast-Planner `plan_env`.
