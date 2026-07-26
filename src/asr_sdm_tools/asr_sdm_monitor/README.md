# README

[toc]

The interface is organized into 3 main modules:

- **Hardware**: run the integrated CPU, memory, disk, and network monitors and visualize `/diagnostics`; compatible external NTP diagnostics can also be displayed
- **Video**: display `/perception*` image streams in selectable video windows
- **Plot**: distinguish normal ROS 2 publishers from active `ros2 bag play` sources, visualize selected live data, record selected topics, and open rosbag / MCAP data independently for playback

The top bar provides **Theme** and **Language** selectors. 

### Quick start

Target environment: Ubuntu 24.04, ROS 2 Jazzy, and Qt 6. Run the following commands from the root of a ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt-get install ros-jazzy-diagnostic-updater
```

If the Qt 6 development and QML runtime modules are not installed, install them explicitly:

```bash
sudo apt update
sudo apt install qt6-base-dev qt6-declarative-dev \
  qml6-module-qtquick qml6-module-qtquick-controls \
  qml6-module-qtquick-layouts qml6-module-qtquick-window \
  qml6-module-qtqml-workerscript
```

Build and run:

```bash
colcon build --packages-select asr_sdm_monitor --event-handlers console_direct+
source install/setup.bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

When moving the source tree between computers or after changing build dependencies, a clean package rebuild can be used:

```bash
rm -rf build/asr_sdm_monitor install/asr_sdm_monitor
colcon build --packages-select asr_sdm_monitor --event-handlers console_direct+
source install/setup.bash
```

The CPU, memory, disk, and network monitor nodes are integrated into the same executable. A separate `ros2_system_monitor` package, folder, launch file, or second startup command is not required.

The installed `config/system_monitor.yaml` file is loaded automatically.

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

#### Runtime architecture

| Execution domain | Main responsibilities |
|---|---|
| **Qt GUI thread** | QML pages, chart rendering, batched plot refresh, final video display, lightweight diagnostics handling, and ROS graph discovery |
| **Plot / Record executor** | Typed plot subscriptions, publisher-GID source filtering, numeric field extraction, generic recording subscriptions, and rosbag2 writing |
| **Video executor** | `Image` / `CompressedImage` subscriptions and image decoding |
| **Hardware executor** | Integrated CPU, memory, disk, and network monitor nodes |

All worker executors are stopped and joined before ROS 2 shutdown.

### Hardware

#### External dependencies

The built-in hardware monitors (`cpu_monitor`, `mem_monitor`, `hdd_monitor`, `net_monitor`) call system utilities. Install the packages below on the host as needed.

##### lm-sensors (`sensors`)

Used for **CPU** core temperature when `check_core_temps` is enabled, and for **HDD** hardware temperature when `no_hw_temp` is set to `false`.

Install on Ubuntu/Debian:

```bash
sudo apt update
sudo apt install lm-sensors
sudo sensors-detect
```

During `sensors-detect`, accept the defaults (press Enter) so the recommended kernel modules are loaded.

Enable the service and verify:

```bash
sudo systemctl enable lm-sensors.service
sudo systemctl start lm-sensors.service
sensors
```

If `sensors` prints CPU or other chip temperatures, installation succeeded.

Related settings in `config/system_monitor.yaml`:

| Monitor | Parameter | Default | Meaning |
|---|---|---|---|
| CPU | `check_core_temps` | `true` | Read core temperatures via `sensors` |
| HDD | `no_hw_temp` | `true` | Skip HW temperature (`sensors -j`); set to `false` to enable |

If `lm-sensors` is not installed, temperature checks may fail while usage, memory, disk, and network metrics continue to work. On VMs or machines without hardware sensors, empty or missing `sensors` output is normal.

#### Function

The CPU, memory, disk, and network monitor nodes are built into `asr_sdm_monitor`. Starting the application with the following single command starts both the Qt/QML interface and all integrated hardware monitors:

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

The integrated monitor nodes publish system status to `/diagnostics`, and the **Hardware** module subscribes to the same topic. No separate `ros2_system_monitor` folder, package, or launch command is needed.

The page parses the following diagnostic categories:

- CPU usage
- Memory usage
- HDD usage
- Network usage
- NTP offset, when compatible NTP diagnostics are published by another ROS 2 node

The module contains five tabs: **CPU**, **Memory**, **HDD**, **Net**, and **NTP**. CPU, memory, disk, and network diagnostics are generated internally. The NTP tab displays compatible NTP diagnostic messages when they are available.

#### CPU page

The **CPU** page shows CPU usage and per-core information.

Summary cards:

| Item | Meaning |
|---|---|
| **Average Usage** | Average CPU usage across all parsed cores |
| **Max Core Usage** | Highest usage among all parsed CPU cores |
| **Average Clock** | Average CPU clock speed |
| **1min Load** | 1-minute load average |
| **5min Load** | 5-minute load average |
| **15min Load** | 15-minute load average |
| **Core Count** | Number of parsed CPU cores |
| **State** | Diagnostic level and diagnostic message |

Chart:

| Chart | Meaning |
|---|---|
| **CPU Usage History** | Recent average CPU usage history, scaled from 0 to 1 |

Table columns:

| Column | Meaning |
|---|---|
| **Core** | CPU core index |
| **Usage** | Calculated usage of the core |
| **Clock** | Core clock speed |
| **User** | User-space CPU percentage from diagnostics |
| **System** | Kernel/system CPU percentage from diagnostics |
| **Idle** | Idle percentage from diagnostics |
| **Status** | Core status text |

#### Memory page

The **Memory** page shows physical memory, swap memory, and memory history.

Summary cards:

| Item | Meaning |
|---|---|
| **Physical Used** | Used physical memory |
| **Physical Total** | Total physical memory |
| **Physical Free** | Free physical memory |
| **Usage** | Physical memory usage percentage |
| **Swap Used** | Used swap memory |
| **Swap Total** | Total swap memory |
| **Update Status** | Whether the memory information was updated from diagnostics |
| **State** | Diagnostic level and diagnostic message |

Chart:

| Chart | Meaning |
|---|---|
| **Memory Usage History** | Recent memory usage history, scaled from 0 to 1 |

Table columns:

| Column | Meaning |
|---|---|
| **Type** | Memory item type, such as physical, swap, or total |
| **Total** | Total capacity |
| **Used** | Used capacity |
| **Free** | Free capacity |

#### HDD page

The **HDD** page shows disk usage information parsed from diagnostics.

Summary cards:

| Item | Meaning |
|---|---|
| **Disk Count** | Number of parsed disk entries |
| **Max Usage** | Highest disk usage among all entries |
| **Status Level** | Diagnostic level |
| **Status Description** | Diagnostic message |

Table columns:

| Column | Meaning |
|---|---|
| **Disk** | Disk or filesystem name |
| **Mount** | Mount point |
| **Size** | Total size |
| **Available** | Available capacity |
| **Use** | Usage percentage |
| **Status** | Disk status |

#### Net page

The **Net** page shows network traffic, interface status, and traffic history.

Summary cards:

| Item | Meaning |
|---|---|
| **Total Input** | Total input traffic rate |
| **Total Output** | Total output traffic rate |
| **Interface Count** | Number of parsed network interfaces |
| **Error Count** | Total receive and transmit errors |
| **Interface List** | Names of detected interfaces |
| **Status Level** | Diagnostic level |
| **Status Description** | Diagnostic message |

Chart:

| Chart | Meaning |
|---|---|
| **Input History** | Recent input traffic history |
| **Output History** | Recent output traffic history |
| **Current Scale** | Adaptive Y-axis scale in MB/s |

Table columns:

| Column | Meaning |
|---|---|
| **Interface** | Network interface name |
| **State** | Interface state |
| **Input** | Current input traffic rate |
| **Output** | Current output traffic rate |
| **RxErr** | Receive error count |
| **TxErr** | Transmit error count |
| **TotalRx** | Total received data |
| **TotalTx** | Total transmitted data |

#### NTP page

The **NTP** page shows time synchronization information parsed from diagnostics.

Summary cards:

| Item | Meaning |
|---|---|
| **Offset** | Current NTP offset in microseconds |
| **Tolerance** | Allowed offset tolerance |
| **Error Tolerance** | Error-level offset tolerance |
| **State** | Diagnostic level and diagnostic message |

Table columns:

| Column | Meaning |
|---|---|
| **Name** | NTP diagnostic item |
| **Value** | Diagnostic value |


#### Typical workflow

1. From a sourced workspace, start the complete monitor application:

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

2. Open **Hardware** from the sidebar.

3. Use the **CPU** page as a typical example.

   ![Hardware CPU page](docs/images/hardware_cpu_overview.png)

4. If the page does not update, inspect `/diagnostics` from another sourced terminal:

```bash
ros2 topic echo /diagnostics --once
```

### Video

#### Function

The **Video** module displays image streams from ROS 2 perception topics. The implementation automatically scans the ROS graph and lists topics that satisfy both conditions:

- The topic name starts with `/perception`
- The topic type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`

Each selected topic is subscribed directly by the monitor and rendered in the corresponding video window. Image display uses aspect-ratio preserving scaling, so the full image remains visible when the window is resized.

#### Buttons and controls

| Control | Function |
|---|---|
| **Video Windows** | Select how many video windows are displayed |
| **1 / 2 / 3 / 4** | Display 1, 2, 3, or 4 video windows |
| **Topic** | Select the image topic for a video window |
| **None** | Disable the video stream for that window |
| **Current Topic** | Show the currently selected topic of the window |
| **Status** | Show whether the window is waiting for frames, receiving images, or has no selected topic |

#### Adjustable parameters

| Parameter | Options / Range | Default | Description |
|---|---:|---:|---|
| **Video Windows** | 1, 2, 3, 4 | 2 | Number of active video windows |
| **Topic** | `None` or discovered `/perception*` image topics | `None` | Image topic shown in each window |

#### Video window behavior

- When no topic is selected, the window displays **Select a /perception* topic to start streaming**.
- When a topic is selected but no image has arrived yet, the window displays **Waiting for video frame ...**.
- When a valid image arrives, the frame is rendered inside the window.
- If the same topic is selected in another window, the previous window is cleared so that one topic is displayed by only one active window.
- If the selected topic disappears from the ROS graph, the monitor clears that video slot and returns it to **None**.
- Hidden windows are cleared automatically when the number of video windows is reduced.

#### Supported image encodings

The monitor supports common raw image encodings handled by the implementation, including:

| Encoding type | Behavior |
|---|---|
| `rgb8` | Displayed as RGB image |
| `bgr8` | Converted to RGB before display |
| `mono8`, `8UC1` | Displayed as grayscale image |
| `rgba8` | Displayed as RGBA image |
| `bgra8` | Converted to RGBA before display |
| Other encodings | Reported as unsupported in the video status |


#### Typical workflow

1. Start the ROS 2 nodes that publish image topics under `/perception*`.

2. Start `asr_sdm_monitor` and open **Video** from the sidebar.

3. Set **Video Windows** to `1`, `2`, `3`, or `4` as needed.

4. In each active video window, use the **Topic** drop-down menu to select one `/perception*` image topic.

   ![Video multi-window topic display](docs/images/video_windows_4_topics.png)

5. If a window no longer needs to display images, reduce the number of video windows or set the window **Topic** to **None**.

6. If no selectable topic appears, check whether the image topic name starts with `/perception` and whether its type is `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`.


### Plot

#### Function

The **Plot** module visualizes numeric fields from ROS topics and records selected ROS 2 messages. It supports three subpages:

| Subpage | Function |
|---|---|
| **Topics** | Scan active publishers, distinguish normal ROS 2 sources from active `ros2 bag play` sources, sort the list, and select the source used by Live plotting and recording |
| **Live** | Plot supported numeric fields from selected topic sources and record every selected topic, including non-plottable message types |
| **Recorded** | Open a rosbag / MCAP directory, read all supported plot fields directly from the bag, and replay them independently of Topics-page selections |

The Plot module can work as a time-series plot or as an XY plot:

- **Time-series plot**: X axis is time, Y axis is one or more topic message fields
- **XY plot**: X axis is a selected topic message field, Y axis is one or more topic message fields

For example, if the X axis is `angular_velocity.x` and the Y axis is `angular_velocity.y`, the chart shows the XY relation between the two IMU angular velocity components.

#### Supported topic types

The following message types can be decoded into numeric fields for **Live** and **Recorded** plotting. Recording is not restricted to this table: every selected topic source with a valid message type can be stored through a generic serialized subscription.

| Message type | Plottable fields |
|---|---|
| `std_msgs/msg/Bool` | topic value, displayed as 0 or 1 |
| `std_msgs/msg/Float32` | topic value |
| `std_msgs/msg/Float64` | topic value |
| `std_msgs/msg/Int8` | topic value |
| `std_msgs/msg/Int16` | topic value |
| `std_msgs/msg/Int32` | topic value |
| `std_msgs/msg/Int64` | topic value |
| `std_msgs/msg/UInt8` | topic value |
| `std_msgs/msg/UInt16` | topic value |
| `std_msgs/msg/UInt32` | topic value |
| `std_msgs/msg/UInt64` | topic value |
| `sensor_msgs/msg/Imu` | `angular_velocity.x/y/z`, `linear_acceleration.x/y/z` |
| `sensor_msgs/msg/Temperature` | `temperature`, `variance` |
| `sensor_msgs/msg/FluidPressure` | `fluid_pressure`, `variance` |
| `sensor_msgs/msg/RelativeHumidity` | `relative_humidity`, `variance` |
| `sensor_msgs/msg/MagneticField` | `magnetic_field.x/y/z` |
| `sensor_msgs/msg/BatteryState` | `voltage`, `temperature`, `current`, `charge`, `capacity`, `design_capacity`, `percentage` |
| `geometry_msgs/msg/Vector3` | `x`, `y`, `z` |
| `geometry_msgs/msg/Vector3Stamped` | `x`, `y`, `z` |
| `geometry_msgs/msg/Twist` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/TwistStamped` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/Accel` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/AccelStamped` | `linear.x/y/z`, `angular.x/y/z` |

#### Topics subpage

The **Topics** subpage lists active ROS graph publisher sources used by **Live** plotting and recording.

Buttons and controls:

| Control | Function |
|---|---|
| **Refresh** | Scan the ROS graph again and refresh publisher sources |
| **Sort** | Sort by `Name A–Z`, `Name Z–A`, `Source`, or `Plottable first` |
| **Checkbox** | Select or clear one publisher source |
| **Topic** | Display the ROS topic name |
| **Type** | Display the ROS message type |
| **Source** | Show `ROS 2 Live` or `ROS 2 Bag Play`, followed by publisher node names when available |
| **Capability** | Show whether the source is plottable, recordable, or recordable only |

Source and selection behavior:

- Normal publishers and active `ros2 bag play` publishers are separated using ROS graph endpoint information and publisher GIDs.
- Multiple rows may therefore have the same topic name but different sources.
- Only one source can be selected for the same topic name. Selecting another source with that name automatically clears the previous source.
- Different topic names can be selected independently.
- Selected supported types provide fields to **Live**; selected unsupported types remain recordable.
- Changing the selected source set clears current Live samples and refreshes plot and recording subscriptions.
- This page does not contain offline topics from a bag opened in **Recorded**.

#### Live subpage

The **Live** subpage displays real-time samples from the source selections made in **Topics**.

Buttons and controls:

| Control | Function |
|---|---|
| **Recording Bag** | Path where a new recording bag will be saved |
| **Start Recording** | Start recording every selected topic source |
| **Stop Recording** | Stop the active recording and close the bag writer |
| **X Axis settings** | Configure the X-axis data source and display style |
| **Y Axis settings** | Configure the number of curves and each curve style |
| **Reset** | Reset the chart view after zooming or panning |

Live and recording behavior:

- Live field selectors contain only supported numeric fields from the selected topic sources.
- The default recording directory is `$HOME/asr_sdm_monitor_recordings`.
- The default bag name format is `plot_yyyyMMdd_HHmmss`.
- If the path field is empty, the monitor generates a default path.
- If the target path already exists, recording will not start.
- Recording stores all selected topic sources, including message types that cannot be plotted.
- Publisher-GID filtering keeps messages from the selected source when the same topic name is published by both a live node and `ros2 bag play`.
- The status text shows the recording path and message count while recording.

#### Recorded subpage

The **Recorded** subpage loads and replays rosbag / MCAP data without depending on the **Topics** checkboxes.

Buttons and controls:

| Control | Function |
|---|---|
| **Recorded Bag** | Path of the recorded bag / MCAP directory |
| **Open** | Open a folder dialog and choose a bag directory |
| **Play** | Start playback from the current time |
| **Pause** | Pause playback |
| **Start Time** | Set the start boundary of playback |
| **End Time** | Set the end boundary of playback |
| **Current Time** | Set the current playback time |
| **Speed** | Set playback speed |
| **Playback slider** | Drag to adjust the current playback time |

When **Open** succeeds, the monitor reads the bag metadata and messages, exposes every supported numeric field found in the bag, and makes all decoded plot samples available immediately. Offline bag topics do not need to be selected in **Topics**, and they are not added to the Topics list.

Adjustable playback parameters:

| Parameter | Options / Range | Description |
|---|---:|---|
| **Start Time** | Inside the bag time range | Playback cannot go before this time |
| **End Time** | Inside the bag time range | Playback stops at this time |
| **Current Time** | Between Start Time and End Time | Current replay position |
| **Speed** | 0.25x, 0.5x, 1.0x, 2.0x, 4.0x in the UI | Playback speed multiplier |

Time input formats:

| Format | Meaning |
|---|---|
| `HH:MM:SS.mmm` | Absolute wall-clock time on the same date as the current fallback time |
| Large numeric timestamp | Treated as absolute milliseconds |
| Small numeric value | Treated as seconds relative to playback start time |

Playback behavior:

- Loading a bag switches the plot data source to **Recorded**.
- Unsupported message types remain in the bag but are skipped by the plot decoder.
- When playback reaches **End Time**, it stops automatically.
- If **Play** is pressed after playback has reached the end, playback restarts from **Start Time**.
- The chart shows a vertical playback marker in time-series mode.
- The playback slider and **Current Time** field update the visible time window.

#### X Axis settings

| Parameter | Options / Range | Default | Description |
|---|---:|---:|---|
| **Label type** | `Time`, `Topic Message` | `Time` | Select whether the X axis uses time or a topic field |
| **Label** | Available plottable field for the active data source | First available field when needed | Used only when **Label type** is `Topic Message` |
| **Show tick labels** | `On`, `Off` | `On` | Show or hide X-axis tick labels |
| **Timestamp format** | `Relative Time`, `Absolute Time` | `Relative Time` | Used only when **Label type** is `Time` |
| **Current Time** | Display only in Live; editable through playback controls in Recorded | Current live or playback time | Shows current reference time |
| **Time window** | Positive number, minimum 0.05 s | 4.00 s | Time span shown in time-series mode |

#### Y Axis settings

| Parameter | Options / Range | Default | Description |
|---|---:|---:|---|
| **Series number** | 1 to 16 | 1 | Number of curves to draw |
| **Show tick labels** | `On`, `Off` | `On` | Show or hide Y-axis tick labels |
| **Series Label** | `None` or an available plottable field | `None` | Field used by this curve |
| **Series Color** | Color dialog or text value | Automatic palette color | Color of the curve |
| **Line width** | Positive number, minimum 0.1 | 1.0 | Width of the curve line |

Series behavior:

- Each Y-axis series has independent **Label**, **Color**, and **Line width** controls.
- The same topic field cannot be selected by multiple Y-axis series at the same time.
- If a selected field disappears, the corresponding series is cleared.
- Only series with valid fields are drawn.

#### Chart mouse operations

| Operation | Function |
|---|---|
| Mouse wheel | Zoom in or out around the mouse position |
| Left mouse drag | Pan the chart view |
| **Reset** | Restore the automatic chart view |

#### Axis scale

| Mode | Function |
|---|---|
| **Independent** | X and Y axes scale independently |
| **Square** | In XY mode, X and Y use the same numeric range so geometric shape is preserved |

#### Typical workflow

1. Open **Plot** and go to **Topics**.

2. Refresh the list when needed, choose a sorting mode, and select the required source for each topic name.

   ![Plot topic selection](docs/images/plot_topics_selection.png)

3. Open **Live**. Supported selected topics provide plot fields; all selected topics can be recorded.

4. For a time-series plot, set **X Axis / Label type** to `Time`, choose **Relative Time** or **Absolute Time**, and adjust **Time window**.

   ![Plot Live time-series](docs/images/plot_live_timeseries.png)

5. For an XY plot, set **X Axis / Label type** to `Topic Message`, then choose the X-axis topic field.

   ![Plot Live topic-series](docs/images/plot_live_topics.png)

6. Set **Series number** and select one Y-axis field for each curve. Adjust color and line width as needed.

7. To record, Topics-page selection is required, then set **Recording Bag**, click **Start Recording**, and click **Stop Recording** when finished.

8. To inspect an offline bag, open **Recorded**, click **Open**, and select a rosbag / MCAP directory. No Topics-page selection is required.

9. Select fields from the opened bag, configure **Start Time**, **End Time**, **Current Time**, and **Speed**, and click **Play**.

   ![Plot Recorded playback](docs/images/plot_recorded_playback.png)

---

### 快速开始

目标环境为 Ubuntu 24.04、ROS 2 Jazzy 和 Qt 6。请在 ROS 2 workspace 根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

如果尚未安装 Qt 6 开发包和 QML 运行模块，可显式安装：

```bash
sudo apt update
sudo apt install qt6-base-dev qt6-declarative-dev \
  qml6-module-qtquick qml6-module-qtquick-controls \
  qml6-module-qtquick-layouts qml6-module-qtquick-window \
  qml6-module-qtqml-workerscript
```

编译并启动：

```bash
colcon build --packages-select asr_sdm_monitor --event-handlers console_direct+
source install/setup.bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

在不同电脑之间移动源码或修改编译依赖后，可以仅清理本 package 再重新编译：

```bash
rm -rf build/asr_sdm_monitor install/asr_sdm_monitor
colcon build --packages-select asr_sdm_monitor --event-handlers console_direct+
source install/setup.bash
```

CPU、内存、磁盘和网络监测节点已经集成到同一个可执行程序中，不需要额外准备 `ros2_system_monitor` 文件夹、package 或 launch 文件，也不需要第二条启动命令。

安装后的 `config/system_monitor.yaml` 会自动加载，也可以显式指定：

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor --ros-args \
  --params-file $(ros2 pkg prefix asr_sdm_monitor)/share/asr_sdm_monitor/config/system_monitor.yaml
```

#### 运行架构

| 执行域 | 主要职责 |
|---|---|
| **Qt GUI 主线程** | QML 页面、曲线最终绘制、Plot 数据批量刷新、视频最终显示、轻量 diagnostics 处理和 ROS graph 话题发现 |
| **Plot / Record executor** | 类型化绘图订阅、publisher GID 来源过滤、数值字段提取、通用录制订阅和 rosbag2 写入 |
| **Video executor** | `Image` / `CompressedImage` 订阅和图像解码 |
| **Hardware executor** | 内置 CPU、内存、磁盘和网络监测节点 |

程序退出时会先停止并回收所有 ROS worker executor，再执行 ROS 2 shutdown。

### Hardware

#### 外部依赖

内置 hardware monitor 节点（`cpu_monitor`、`mem_monitor`、`hdd_monitor`、`net_monitor`）会调用系统命令，请按需安装下列工具。

##### lm-sensors（`sensors`）

在启用 CPU 核心温度（`check_core_temps`）或 HDD 硬件温度（`no_hw_temp: false`）时使用。

Ubuntu/Debian 安装：

```bash
sudo apt update
sudo apt install lm-sensors
sudo sensors-detect
```

运行 `sensors-detect` 时按提示选择 **Yes**（通常直接回车即可），以加载推荐的内核模块。

启用服务并验证：

```bash
sudo systemctl enable lm-sensors.service
sudo systemctl start lm-sensors.service
sensors
```

若 `sensors` 能输出 CPU 或其他芯片温度，说明安装成功。

相关配置见 `config/system_monitor.yaml`：

| Monitor | 参数 | 默认值 | 说明 |
|---|---|---|---|
| CPU | `check_core_temps` | `true` | 通过 `sensors` 读取核心温度 |
| HDD | `no_hw_temp` | `true` | 跳过硬件温度（`sensors -j`）；设为 `false` 可启用 |

未安装 `lm-sensors` 时，温度相关诊断可能失败，内存、磁盘、网络等其余指标仍可正常显示。虚拟机或无硬件传感器的机器上 `sensors` 无输出属于正常现象。

#### 功能

CPU、内存、磁盘和网络监测节点已经内置在 `asr_sdm_monitor` 中。执行下面一个命令即可同时启动 Qt/QML 界面和全部内置硬件监测：

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

内置节点会把系统状态发布到 `/diagnostics`，**Hardware** 模块订阅同一个话题并显示结果。不需要单独的 `ros2_system_monitor` 文件夹、package 或启动命令。

页面会解析以下诊断类别：

- CPU usage
- Memory usage
- HDD usage
- Network usage
- NTP offset：当其他 ROS 2 节点发布兼容的 NTP diagnostics 时显示

该模块包含五个子页面：**CPU**、**Memory**、**HDD**、**Net**、**NTP**。CPU、内存、磁盘和网络诊断由本程序内部生成；NTP 页面在收到兼容的 NTP 诊断消息时显示对应信息。

#### CPU 页面

**CPU** 页面显示 CPU 占用、负载、主频和每个核心的详细信息。

概要卡片：

| 项目 | 含义 |
|---|---|
| **Average Usage** | 所有解析到的 CPU 核心的平均占用率 |
| **Max Core Usage** | 所有 CPU 核心中的最高占用率 |
| **Average Clock** | 平均 CPU 主频 |
| **1min Load** | 1 分钟平均负载 |
| **5min Load** | 5 分钟平均负载 |
| **15min Load** | 15 分钟平均负载 |
| **Core Count** | 解析到的 CPU 核心数量 |
| **State** | diagnostic level 和 diagnostic message |

曲线：

| 曲线 | 含义 |
|---|---|
| **CPU Usage History** | 最近一段时间的 CPU 平均占用率历史，纵轴范围 0 到 1 |

表格列：

| 列名 | 含义 |
|---|---|
| **Core** | CPU 核心编号 |
| **Usage** | 计算得到的核心占用率 |
| **Clock** | 核心主频 |
| **User** | diagnostics 中的用户态 CPU 百分比 |
| **System** | diagnostics 中的系统态 CPU 百分比 |
| **Idle** | diagnostics 中的空闲百分比 |
| **Status** | 核心状态文本 |

#### Memory 页面

**Memory** 页面显示物理内存、交换分区和内存占用历史。

概要卡片：

| 项目 | 含义 |
|---|---|
| **Physical Used** | 已使用的物理内存 |
| **Physical Total** | 物理内存总量 |
| **Physical Free** | 空闲物理内存 |
| **Usage** | 物理内存使用率 |
| **Swap Used** | 已使用的交换分区 |
| **Swap Total** | 交换分区总量 |
| **Update Status** | 内存信息是否从 diagnostics 更新 |
| **State** | diagnostic level 和 diagnostic message |

曲线：

| 曲线 | 含义 |
|---|---|
| **Memory Usage History** | 最近一段时间的内存使用率历史，纵轴范围 0 到 1 |

表格列：

| 列名 | 含义 |
|---|---|
| **Type** | 内存类型，例如 physical、swap 或 total |
| **Total** | 总量 |
| **Used** | 已用量 |
| **Free** | 空闲量 |

#### HDD 页面

**HDD** 页面显示磁盘使用信息。

概要卡片：

| 项目 | 含义 |
|---|---|
| **Disk Count** | 解析到的磁盘条目数量 |
| **Max Usage** | 所有磁盘条目中的最高使用率 |
| **Status Level** | diagnostic level |
| **Status Description** | diagnostic message |

表格列：

| 列名 | 含义 |
|---|---|
| **Disk** | 磁盘或文件系统名称 |
| **Mount** | 挂载点 |
| **Size** | 总容量 |
| **Available** | 可用容量 |
| **Use** | 使用率 |
| **Status** | 磁盘状态 |

#### Net 页面

**Net** 页面显示网络流量、网络接口状态和流量历史。

概要卡片：

| 项目 | 含义 |
|---|---|
| **Total Input** | 总输入流量速率 |
| **Total Output** | 总输出流量速率 |
| **Interface Count** | 解析到的网络接口数量 |
| **Error Count** | 接收错误和发送错误总数 |
| **Interface List** | 检测到的网络接口名称 |
| **Status Level** | diagnostic level |
| **Status Description** | diagnostic message |

曲线：

| 曲线 | 含义 |
|---|---|
| **Input History** | 输入流量历史 |
| **Output History** | 输出流量历史 |
| **Current Scale** | 自适应纵轴量程，单位 MB/s |

表格列：

| 列名 | 含义 |
|---|---|
| **Interface** | 网络接口名称 |
| **State** | 接口状态 |
| **Input** | 当前输入流量速率 |
| **Output** | 当前输出流量速率 |
| **RxErr** | 接收错误数 |
| **TxErr** | 发送错误数 |
| **TotalRx** | 累计接收数据 |
| **TotalTx** | 累计发送数据 |

#### NTP 页面

**NTP** 页面显示时间同步相关信息。

概要卡片：

| 项目 | 含义 |
|---|---|
| **Offset** | 当前 NTP 偏移，单位微秒 |
| **Tolerance** | 允许的偏移容差 |
| **Error Tolerance** | Error 级别的偏移容差 |
| **State** | diagnostic level 和 diagnostic message |

表格列：

| 列名 | 含义 |
|---|---|
| **Name** | NTP 诊断项目 |
| **Value** | 对应的诊断值 |



#### 典型使用流程

1. 在已经 source 的 workspace 中启动完整 monitor：

```bash
ros2 run asr_sdm_monitor asr_sdm_monitor
```

2. 在侧边栏打开 **Hardware**。

3. 以 **CPU** 页面作为典型示例。

   ![Hardware CPU 页面](docs/images/hardware_cpu_overview.png)

4. 如果页面没有刷新，可以在另一个已 source 的终端中检查 `/diagnostics`：

```bash
ros2 topic echo /diagnostics --once
```

### Video

#### 功能

**Video** 模块用于显示 ROS 2 图像话题。程序会自动扫描 ROS graph，并列出同时满足下面两个条件的话题：

- 话题名以 `/perception` 开头
- 话题类型是 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`

每个窗口会直接订阅所选图像话题并渲染图像。图像显示采用保持长宽比的缩放方式，因此窗口大小变化时不会拉伸变形。

#### 按钮和控件

| 控件 | 作用 |
|---|---|
| **Video Windows** | 选择显示几个视频窗口 |
| **1 / 2 / 3 / 4** | 显示 1、2、3 或 4 个视频窗口 |
| **Topic** | 为某个视频窗口选择图像话题 |
| **None** | 关闭该窗口的视频显示 |
| **Current Topic** | 显示该窗口当前选择的话题 |
| **Status** | 显示该窗口是等待图像、正在接收图像，还是没有选择话题 |

#### 可以设置的参数

| 参数 | 选项 / 范围 | 默认值 | 说明 |
|---|---:|---:|---|
| **Video Windows** | 1, 2, 3, 4 | 2 | 当前显示的视频窗口数量 |
| **Topic** | `None` 或扫描到的 `/perception*` 图像话题 | `None` | 每个窗口显示的图像话题 |

#### 视频窗口行为

- 没有选择话题时，窗口显示 **Select a /perception* topic to start streaming**。
- 选择了话题但还没有收到图像时，窗口显示 **Waiting for video frame ...**。
- 收到有效图像后，图像会显示在窗口中。
- 如果同一个话题被选择到另一个窗口，原来的窗口会被清空，避免一个话题同时占用多个窗口。
- 如果所选话题从 ROS graph 中消失，该窗口会自动恢复为 **None**。
- 当视频窗口数量减少时，被隐藏的窗口会自动清空。

#### 支持的图像编码

monitor 支持实现中处理的常见 raw image encoding：

| Encoding 类型 | 行为 |
|---|---|
| `rgb8` | 作为 RGB 图像显示 |
| `bgr8` | 转换为 RGB 后显示 |
| `mono8`, `8UC1` | 作为灰度图显示 |
| `rgba8` | 作为 RGBA 图像显示 |
| `bgra8` | 转换为 RGBA 后显示 |
| 其他 encoding | 在视频状态中显示为 unsupported |

#### 典型使用流程

1. 启动会发布 `/perception*` 图像话题的 ROS 2 节点。

2. 启动 `asr_sdm_monitor`，并在侧边栏打开 **Video**。

3. 根据需要把 **Video Windows** 设置为 `1`、`2`、`3` 或 `4`。

4. 在每个启用的视频窗口中，通过 **Topic** 下拉菜单选择一个 `/perception*` 图像话题。

   ![Video 双窗口话题显示](docs/images/video_windows_4_topics.png)

5. 如果某个窗口不再需要显示图像，可以减少窗口数量，或把该窗口的 **Topic** 设为 **None**。

6. 如果没有出现可选话题，检查图像话题名称是否以 `/perception` 开头，并确认类型是 `sensor_msgs/msg/Image` 或 `sensor_msgs/msg/CompressedImage`。


### Plot

#### 功能

**Plot** 模块用于可视化 ROS topic 中的数值字段，并录制所选 ROS 2 消息。它包含三个子页面：

| 子页面 | 功能 |
|---|---|
| **Topics** | 扫描当前 publisher，区分普通 ROS 2 来源与正在运行的 `ros2 bag play` 来源，对列表排序，并选择 Live 绘图和录制使用的来源 |
| **Live** | 绘制所选来源中的受支持数值字段，并录制全部已选话题，包括不可绘制的消息类型 |
| **Recorded** | 打开 rosbag / MCAP 目录，直接读取 bag 内所有受支持的绘图字段，且回放与 Topics 页面勾选完全解耦 |

Plot 模块支持两种绘图方式：

- **时间序列图**：X 轴是时间，Y 轴是一个或多个 topic message 字段
- **XY 图**：X 轴是一个 topic message 字段，Y 轴是一个或多个 topic message 字段

例如 X 轴选择 `angular_velocity.x`，Y 轴选择 `angular_velocity.y` 时，图中显示 IMU 角速度两个分量之间的 XY 关系。

#### 支持的话题类型

下表中的消息类型可以在 **Live** 和 **Recorded** 中解析为数值字段。录制不受此表限制：只要 Topics 中选择的话题具有有效消息类型，就可以通过通用序列化订阅写入 bag。

| Message type | 可绘制字段 |
|---|---|
| `std_msgs/msg/Bool` | topic value，以 0 或 1 显示 |
| `std_msgs/msg/Float32` | topic value |
| `std_msgs/msg/Float64` | topic value |
| `std_msgs/msg/Int8` | topic value |
| `std_msgs/msg/Int16` | topic value |
| `std_msgs/msg/Int32` | topic value |
| `std_msgs/msg/Int64` | topic value |
| `std_msgs/msg/UInt8` | topic value |
| `std_msgs/msg/UInt16` | topic value |
| `std_msgs/msg/UInt32` | topic value |
| `std_msgs/msg/UInt64` | topic value |
| `sensor_msgs/msg/Imu` | `angular_velocity.x/y/z`, `linear_acceleration.x/y/z` |
| `sensor_msgs/msg/Temperature` | `temperature`, `variance` |
| `sensor_msgs/msg/FluidPressure` | `fluid_pressure`, `variance` |
| `sensor_msgs/msg/RelativeHumidity` | `relative_humidity`, `variance` |
| `sensor_msgs/msg/MagneticField` | `magnetic_field.x/y/z` |
| `sensor_msgs/msg/BatteryState` | `voltage`, `temperature`, `current`, `charge`, `capacity`, `design_capacity`, `percentage` |
| `geometry_msgs/msg/Vector3` | `x`, `y`, `z` |
| `geometry_msgs/msg/Vector3Stamped` | `x`, `y`, `z` |
| `geometry_msgs/msg/Twist` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/TwistStamped` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/Accel` | `linear.x/y/z`, `angular.x/y/z` |
| `geometry_msgs/msg/AccelStamped` | `linear.x/y/z`, `angular.x/y/z` |

#### Topics 子页面

**Topics** 子页面只展示当前 ROS graph 中的 publisher 来源，用于 **Live** 绘图和录制。

按钮和控件：

| 控件 | 作用 |
|---|---|
| **Refresh** | 重新扫描 ROS graph 并刷新 publisher 来源 |
| **Sort** | 按 `名称正序 A–Z`、`名称倒序 Z–A`、`话题来源` 或 `可绘图优先` 排序 |
| **Checkbox** | 选择或取消一个 publisher 来源 |
| **Topic** | 显示 ROS topic 名称 |
| **Type** | 显示 ROS message type |
| **Source** | 显示 `ROS 2 Live` 或 `ROS 2 Bag Play`，可用时同时显示 publisher node 名称 |
| **Capability** | 显示可绘图且可录制，或仅可录制 |

来源与选择规则：

- 程序通过 ROS graph endpoint 信息和 publisher GID 区分普通 publisher 与正在运行的 `ros2 bag play` publisher。
- 因此列表中可能出现名称相同但来源不同的多行话题。
- 同一个话题名称只能选择一个来源；勾选另一个同名来源时，原来源会自动取消。
- 不同名称的话题可以分别勾选。
- 已选且受支持的话题会向 **Live** 提供绘图字段；已选但不支持绘图的话题仍然可以录制。
- 修改选择后会清空当前 Live 样本，并刷新绘图订阅和录制订阅。
- **Recorded** 中打开的离线 bag 话题不会出现在 Topics 页面。

#### Live 子页面

**Live** 子页面显示 Topics 中所选来源的实时数据。

按钮和控件：

| 控件 | 作用 |
|---|---|
| **Recording Bag** | 新录制 bag 的保存路径 |
| **Start Recording** | 开始录制全部已选话题来源 |
| **Stop Recording** | 停止当前录制并关闭 bag writer |
| **X Axis settings** | 设置 X 轴数据来源和显示方式 |
| **Y Axis settings** | 设置曲线数量和每条曲线的显示方式 |
| **Reset** | 鼠标缩放或拖动后恢复自动视野 |

Live 与录制行为：

- Live 字段下拉菜单只包含已选来源中受支持的数值字段。
- 默认录制目录是 `$HOME/asr_sdm_monitor_recordings`。
- 默认 bag 名称格式是 `plot_yyyyMMdd_HHmmss`。
- 如果路径为空，monitor 会自动生成默认路径。
- 如果目标路径已经存在，录制不会开始。
- 录制会保存全部已选话题，包括不能绘制的消息类型。
- 当普通节点和 `ros2 bag play` 同时发布同名话题时，publisher GID 过滤只保留所选来源的消息。
- 录制过程中状态栏会显示录制路径和消息数量。

#### Recorded 子页面

**Recorded** 子页面用于加载和回放 rosbag / MCAP 数据，不依赖 **Topics** 页面的勾选状态。

按钮和控件：

| 控件 | 作用 |
|---|---|
| **Recorded Bag** | 已录制 rosbag / MCAP 目录路径 |
| **Open** | 打开文件夹选择对话框，选择 bag 目录 |
| **Play** | 从当前时间开始播放 |
| **Pause** | 暂停播放 |
| **Start Time** | 设置回放起始边界 |
| **End Time** | 设置回放结束边界 |
| **Current Time** | 设置当前回放时间 |
| **Speed** | 设置播放速度 |
| **Playback slider** | 拖动调整当前回放时间 |

**Open** 成功后，程序会读取 bag 元数据和消息，立即把 bag 中所有受支持的数值字段加入 Recorded 的字段选择，并提供全部已解码样本。离线 bag 话题不需要在 Topics 中勾选，也不会加入 Topics 列表。

可设置的回放参数：

| 参数 | 选项 / 范围 | 说明 |
|---|---:|---|
| **Start Time** | bag 时间范围内 | 回放不会早于该时间 |
| **End Time** | bag 时间范围内 | 回放到该时间后停止 |
| **Current Time** | Start Time 和 End Time 之间 | 当前回放位置 |
| **Speed** | UI 中提供 0.25x、0.5x、1.0x、2.0x、4.0x | 回放速度倍率 |

时间输入格式：

| 格式 | 含义 |
|---|---|
| `HH:MM:SS.mmm` | 使用当前 fallback date 上的绝对时刻 |
| 很大的数字时间戳 | 作为绝对毫秒时间 |
| 较小的数字 | 作为相对 playback start time 的秒数 |

回放行为：

- bag 加载成功后，Plot 数据源会切换到 **Recorded**。
- 不受支持的消息类型仍保留在原 bag 中，但不会被绘图解码器读取。
- 当回放到 **End Time** 时会自动停止。
- 如果回放已经结束，再点击 **Play**，会从 **Start Time** 重新播放。
- 时间序列图中会显示垂直的当前回放时间标记线。
- 底部进度条和 **Current Time** 字段都会影响当前可视时间窗口。

#### X Axis 设置

| 参数 | 选项 / 范围 | 默认值 | 说明 |
|---|---:|---:|---|
| **Label type** | `Time`, `Topic Message` | `Time` | 选择 X 轴使用时间还是 topic 字段 |
| **Label** | 当前数据源中的可用绘图字段 | 需要时使用第一个可用字段 | 只在 **Label type** 为 `Topic Message` 时使用 |
| **Show tick labels** | `On`, `Off` | `On` | 是否显示 X 轴刻度标签 |
| **Timestamp format** | `Relative Time`, `Absolute Time` | `Relative Time` | 只在 **Label type** 为 `Time` 时使用 |
| **Current Time** | Live 中仅显示；Recorded 中通过回放控件编辑 | 当前实时或回放时间 | 显示当前参考时间 |
| **Time window** | 正数，最小 0.05 s | 4.00 s | 时间序列图中显示的时间跨度 |

#### Y Axis 设置

| 参数 | 选项 / 范围 | 默认值 | 说明 |
|---|---:|---:|---|
| **Series number** | 1 到 16 | 1 | 要绘制的曲线数量 |
| **Show tick labels** | `On`, `Off` | `On` | 是否显示 Y 轴刻度标签 |
| **Series Label** | `None` 或当前数据源中的可用绘图字段 | `None` | 该曲线使用的字段 |
| **Series Color** | 颜色对话框或文本颜色值 | 自动颜色 | 曲线颜色 |
| **Line width** | 正数，最小 0.1 | 1.0 | 曲线线宽 |

曲线行为：

- 每条 Y 轴曲线都可以独立设置 **Label**、**Color** 和 **Line width**。
- 同一个 topic 字段不能同时被多个 Y 轴 series 重复选择。
- 如果已选择字段消失，对应曲线会被清空。
- 只有选择了有效字段的 series 才会被绘制。

#### 曲线鼠标操作

| 操作 | 作用 |
|---|---|
| 鼠标滚轮 | 以鼠标位置为中心放大或缩小曲线视野 |
| 鼠标左键拖动 | 平移曲线视野 |
| Reset | 恢复自动视野 |

#### 坐标轴比例

| 模式 | 作用 |
|---|---|
| **Independent** | X 轴和 Y 轴独立缩放 |
| **Square** | XY 图中让 X 轴和 Y 轴使用相同数值范围，保持几何形状比例 |

#### 典型使用流程

1. 在侧边栏打开 **Plot**，进入 **Topics**。

2. 必要时刷新列表，选择排序方式，并为每个话题名称勾选需要使用的来源。

   ![Plot topic 选择](docs/images/plot_topics_selection.png)

3. 进入 **Live**。受支持的已选话题可用于绘图，全部已选话题都可以录制。

4. 时间序列图中，将 **X Axis / Label type** 设置为 `Time`，选择 **Relative Time** 或 **Absolute Time**，并调整 **Time window**。

   ![Plot Live 时间序列](docs/images/plot_live_timeseries.png)

5. XY 图中，将 **X Axis / Label type** 设置为 `Topic Message`，然后选择 X 轴字段。

   ![Plot Live topic 序列](docs/images/plot_live_topics.png)

6. 设置 **Series number**，为每条曲线选择 Y 轴字段，并按需调整颜色和线宽。

7. 需要录制时，先在 Topics 中勾选录制话题，设置 **Recording Bag**，点击 **Start Recording**，结束后点击 **Stop Recording**。

8. 查看离线 bag 时，进入 **Recorded**，点击 **Open** 并选择 rosbag / MCAP 目录，不需要先在 Topics 中勾选。

9. 从已打开 bag 的字段中选择曲线，设置 **Start Time**、**End Time**、**Current Time** 和 **Speed**，然后点击 **Play**。

   ![Plot Recorded 回放](docs/images/plot_recorded_playback.png)
