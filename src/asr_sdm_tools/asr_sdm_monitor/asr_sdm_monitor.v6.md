# asr_sdm_monitor.v6

+ Plot页面
  + Topics 根据 publisher GID 区分正常 ROS2 话题和 `ros2 bag play` 的话题
  + Topics 增加了话题排序功能：按名称A-Z，按名称Z-A，按话题来源(ROS2 Live/ROS2 play bag)，按是否可绘制
  + Live 页面记录 Topics 勾选的所有话题，允许绘制“ Topics 勾选且可绘制类型”的话题
  + Recorded 点击 Open 后打开数据包，可以直接播放数据。为什么修改：之前需要在 Topics 勾选话才能播放话题，现在与 Topics 页面解耦，打开 Open 数据包即可播放。
+ 多线程
  
  ```
  线程 1：Qt GUI 主线程
  ├── 所有 QML 页面
  ├── Plot 曲线最终绘制
  ├── Plot 数据批量刷新
  ├── 视频最终显示
  ├── diagnostics
  └── ROS Graph 话题发现
  
  线程 2：Plot / Record ROS Executor
  ├── Plot 类型化话题订阅
  ├── publisher GID 来源过滤
  ├── 消息字段提取
  ├── GenericSubscription 录制
  └── rosbag2 Writer 写入
  
  线程 3：视频 ROS Executor
  ├── Image / CompressedImage 订阅
  └── 图像解码
  
  线程 4：硬件监控 ROS Executor
  ├── CPU
  ├── Memory
  ├── Disk
  └── Network
  ```
  
  