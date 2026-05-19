# asr_sdm_camera_realsense_d405

## RealSense SDK and ROS 2 wrapper installation

This package depends on the official RealSense SDK 2.0 and the official
RealSense ROS 2 wrapper. Install them before building this package.

The RealSense SDK accesses the camera hardware. The ROS 2 wrapper exposes the
camera data as ROS 2 topics, services, parameters, and TF.

Choose only one SDK installation method to avoid multiple versions and workspace
conflicts. The commands below follow the official RealSense Linux Debian
installation guide and the official RealSense ROS wrapper README.

### Install RealSense SDK 2.0 on Ubuntu

RealSense SDK 2.0 provides Debian packages for Ubuntu 20.04, 22.04, and 24.04
LTS on Intel X86, AMD64, and ARM-based systems.

##### Register the RealSense APT repository

```bash
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
```

Make sure apt HTTPS support is installed:

```bash
sudo apt-get install apt-transport-https
```

Add the server to the list of repositories:

```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

##### Install SDK packages

```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```

Optional debug package:

```bash
sudo apt-get install librealsense2-dbg
```

Reconnect the RealSense depth camera and run:

```bash
realsense-viewer
```

Verify that the kernel is updated:

```bash
modinfo uvcvideo | grep "version:"
```

The output should include `realsense` in the version string.

### Install RealSense ROS 2 wrapper

The recommended method for normal ROS 2 usage is to install the Debian packages
from ROS servers.

##### Debian package from ROS servers

Replace `<ROS_DISTRO>` with the ROS 2 distribution name, such as `humble` or
`jazzy`.

```bash
sudo apt install ros-<ROS_DISTRO>-realsense2-*
```

For example, for Humble:

```bash
sudo apt install ros-humble-realsense2-*
```

After installation, check that the official wrapper can start:

```bash
ros2 launch realsense2_camera rs_launch.py
```

You can also start the node directly:

```bash
ros2 run realsense2_camera realsense2_camera_node
```

##### Image transport plugins

Install the image transport plugins if you also want compressed image transport
topics such as `compressed` and `compressedDepth` when supported by your ROS 2
installation and wrapper settings.

```bash
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-image-transport-plugins
```

##### Source installation option

Use this option only when the ROS apt package is unavailable or when source-level
changes are required.

Create a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
```

Clone the latest ROS Wrapper for RealSense cameras into `~/ros2_ws/src/`:

```bash
git clone https://github.com/realsenseai/realsense-ros.git -b ros2-master
cd ~/ros2_ws
```

Install dependencies:

```bash
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```

Build:

```bash
colcon build
```

Source environment:

```bash
ROS_DISTRO=<YOUR_SYSTEM_ROS_DISTRO>  # set your ROS_DISTRO: kilted, jazzy, iron, humble, foxy
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```

## Package overview

This package is a project-level ROS 2 wrapper for Intel RealSense D435IF.

It does **not** reimplement the RealSense driver. Instead, it launches the official
`realsense2_camera` package with a project-specific configuration file.

Current default target camera: **Intel RealSense D435IF**.

The package name still contains `d405` for repository compatibility:

```text
asr_sdm_camera_realsense_d405
```

## Function

The package starts the official `realsense2_camera` driver and enables the D435IF
sensor streams and common derived outputs that are useful for downstream ROS 2
nodes:

- color image
- depth image
- infrared 1 image
- infrared 2 image
- camera info
- accel sample
- gyro sample
- combined IMU
- aligned depth image
- RGBD topic
- point cloud
- TF / TF static
- diagnostics
- RealSense metadata and extrinsics topics published by the official driver

The actual topic set depends on the connected device, firmware, installed
`realsense2_camera` version, installed image transport plugins, and the parameters
in `config/d435if.yaml`.

## Package structure

```text
asr_sdm_camera_realsense_d405/
├── CMakeLists.txt
├── package.xml
├── README.md
├── Tutorial_Intel realsense D405 Stro Calibration.readme.md
├── launch/
│   └── camera.launch.py
└── config/
    └── d435if.yaml
```

The old D405 stereo calibration tutorial file is preserved for reference and is
not used by the D435IF launch path.

## Dependencies

This package expects the following official RealSense components to be available:

- RealSense SDK 2.0, provided by `librealsense2`
- RealSense ROS 2 wrapper, provided by `realsense2_camera`
- optional image transport plugins, provided by `image_transport_plugins`

For a normal Ubuntu and ROS 2 installation, follow the installation steps at the
beginning of this README.

## Build

Put this package inside a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cp -r asr_sdm_camera_realsense_d405 ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select asr_sdm_camera_realsense_d405
source install/setup.bash
```

## Launch D435IF

Default launch:

```bash
ros2 launch asr_sdm_camera_realsense_d405 camera.launch.py
```

This loads:

```text
config/d435if.yaml
```

The default node and topic namespace are intentionally kept the same as the official wrapper:

```text
/camera/camera
/camera/camera/...
```

## D435IF topic output

This package enables the D435IF color, depth, infrared, gyro, accel, aligned
depth, RGBD, point cloud, TF, and diagnostics outputs in `config/d435if.yaml`.

The exact topic list must be checked on the target machine after launch:

```bash
ros2 topic list -t | sort
```

##### Typical D435IF topics with the default config

```text
/camera/camera/accel/imu_info
/camera/camera/accel/metadata
/camera/camera/accel/sample
/camera/camera/aligned_depth_to_color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/aligned_depth_to_infra1/camera_info
/camera/camera/aligned_depth_to_infra1/image_raw
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/color/points
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_accel
/camera/camera/extrinsics/depth_to_color
/camera/camera/extrinsics/depth_to_gyro
/camera/camera/extrinsics/depth_to_infra1
/camera/camera/extrinsics/depth_to_infra2
/camera/camera/gyro/imu_info
/camera/camera/gyro/metadata
/camera/camera/gyro/sample
/camera/camera/imu
/camera/camera/infra1/camera_info
/camera/camera/infra1/image_rect_raw
/camera/camera/infra1/metadata
/camera/camera/infra2/camera_info
/camera/camera/infra2/image_rect_raw
/camera/camera/infra2/metadata
/camera/camera/rgbd
/diagnostics
/parameter_events
/rosout
/tf
/tf_static
```

Depending on the installed wrapper version and enabled image transport plugins,
additional transport topics may also appear under image topics, for example:

```text
/camera/camera/color/image_raw/compressed
/camera/camera/depth/image_rect_raw/compressedDepth
/camera/camera/aligned_depth_to_color/image_raw/compressedDepth
```

The supported stream profiles reported by `rs-enumerate-devices -c` are not
separate ROS topics. They are possible formats for the same stream topics.

## Verify data flow

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/gyro/sample
ros2 topic hz /camera/camera/accel/sample
ros2 topic hz /camera/camera/imu
ros2 topic hz /camera/camera/depth/color/points
```

Check IMU content:

```bash
ros2 topic echo /camera/camera/imu
```

Check RGBD topic:

```bash
ros2 topic echo /camera/camera/rgbd
```

Check device services:

```bash
ros2 service list | grep /camera/camera
```

## Notes

- `realsense2_camera` performs the actual sensor access and ROS 2 publishing.
- This package only selects and installs the launch/config files.
- Topic names are currently left as the official defaults.
- To change topic names later, modify `camera_namespace` and `camera_name` in
  `config/d435if.yaml`, or create another config file and pass it with
  `config_file:=...`.
- `enable_rgbd` is enabled to maximize useful RealSense topic output.
- `tf_publish_rate` is set above `0.0` so `/tf` is published in addition to
  `/tf_static`.
- The D405 stereo calibration tutorial is preserved but is not part of the
  D435IF launch configuration.

## Future extension

To support another RealSense model later, add another flat config file, for example:

```text
config/<model>.yaml
```

Then launch it with:

```bash
ros2 launch asr_sdm_camera_realsense_d405 camera.launch.py   config_file:=/absolute/path/to/config/<model>.yaml
```

No new driver code is needed unless the official RealSense wrapper cannot provide
a required stream or message.
