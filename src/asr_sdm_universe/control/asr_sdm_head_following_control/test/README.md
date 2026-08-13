# front-unit-following 离线仿真测试

该目录包含 2D 和 3D 前端跟随控制器的离线仿真程序。

## 编译

```bash
cd ~/asr_sdm_robo
colcon build --packages-up-to asr_sdm_head_following_control
source install/setup.bash
```

## 启动 2D 离线仿真

```bash
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_2d
```

关闭弹出的绘图窗口或按 `Ctrl+C` 退出。

## 启动 3D 离线仿真

```bash
ros2 run asr_sdm_head_following_control front_unit_following_controller_test_3d
```

关闭弹出的绘图窗口或按 `Ctrl+C` 退出。

## 文件作用

| 文件 | 作用 |
|---|---|
| `front_unit_following_controller_test_2d.cpp` | 运行 2D 前端跟随离线仿真，并绘制头部、关节、尾部轨迹和关节角曲线。 |
| `front_unit_following_controller_test_3d.cpp` | 运行 3D 前端跟随离线仿真，并绘制空间 body 轨迹、关节角和跟随误差。 |
| `third_party/matplotlibcpp.h` | 2D 测试使用的 matplotlib C++ 封装头文件。 |
| `third_party/LICENSE.matplotlibcpp` | matplotlibcpp 的许可证文件。 |
| `README.md` | 当前说明文档。 |
