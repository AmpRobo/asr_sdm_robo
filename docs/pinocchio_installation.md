# Pinocchio 安装说明

本文说明 ASR-SDM 工作区使用的 Pinocchio 安装方式。Pinocchio 项目地址：
[stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)。

仓库根目录的 `setup-dev-env.sh --common` 会从源码编译 eigenpy、Coal 和
Pinocchio，并安装到 `/opt/openrobots`。源码与编译目录位于与 `asr_sdm_robo` 并列的 `../dependency/`。

## 1. 支持环境

推荐环境：

- Ubuntu 24.04 amd64；
- ROS 2 Jazzy；
- Python 3；
- 工作区位于 `~/asr_sdm_robo`，或将下列命令中的路径替换为实际路径。

## 2. 安装

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --common
```

如果 ROS 2 Jazzy 尚未安装，可以一并安装：

```bash
./setup-dev-env.sh --ros --common
```

脚本会：

- 通过 apt 安装编译依赖（Eigen、Boost、NumPy、pybind11、urdfdom、tinyxml2、Assimp 等）；
- 按 eigenpy → Coal → Pinocchio 的顺序，把固定版本的源码浅克隆到
  `dependency/<项目>-<版本>/`（只拉取所需的 `cmake/` 子模块）；
- 在各自的 `build/` 中以 Release 模式编译，关闭测试、示例和文档，开启 Python 绑定；
- 安装到 `/opt/openrobots`（不可写时使用 `sudo`），并把相关环境变量写入 `~/.bashrc`。

当前固定版本（定义在 `setup-dev-env.sh` 顶部）：

| 项目 | 版本 |
|---|---|
| eigenpy | 3.13.0 |
| Coal | 3.0.4 |
| Pinocchio | 4.1.0 |

该组合满足各项目 `pixi.toml` 中的约束（Pinocchio 4.1.0 要求 eigenpy >= 3.7.0、
Coal >= 3）。升级时请同时修改三个版本变量并确认约束仍然满足，再运行
`./setup-dev-env.sh --common`；新版本使用新的源码目录，会自动重新编译。

已安装的组件会被跳过。需要强制重新编译时：

```bash
./setup-dev-env.sh --common --rebuild
```

并行度为逻辑 CPU 数量的一半（向下取整，最少 1 个 job），以控制 Python 绑定编译的
内存占用。

常用设置：

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `PINOCCHIO_INSTALL_PREFIX` | `/opt/openrobots` | 三个项目的安装前缀 |
| `PINOCCHIO_BUILD_COLLISION` | `OFF` | 是否启用 Pinocchio 基于 Coal 的碰撞支持 |

修改上述变量后需加 `--rebuild`，例如：

```bash
PINOCCHIO_BUILD_COLLISION=ON ./setup-dev-env.sh --common --rebuild
```

安装完成后加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
```

脚本写入 `~/.bashrc` 的环境变量（`# >>> pinocchio source install setup >>>` 区块）：

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python<当前版本>/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

实际 Python 路径由脚本根据当前解释器版本生成。

`~/.bashrc` 只对之后新开的交互式终端生效。为了让安装前已打开的终端、非交互 shell、
IDE 和 CI 也能找到这套库，脚本还会：

- 在 CMake 用户包注册表 `~/.cmake/packages/{eigenpy,coal,pinocchio}/` 中登记
  `/opt/openrobots/lib/cmake/<包名>`，`find_package()` 默认会查找该注册表；
- 写入 `/etc/ld.so.conf.d/asr-sdm-openrobots.conf` 并执行 `ldconfig`；
- 在 `/usr/local/lib/python3.X/dist-packages/` 写入 `asr-sdm-openrobots.pth`，
  使系统 `python3` 可直接 `import pinocchio`。

旧版脚本通过 robotpkg 安装时写入的
`pinocchio robotpkg setup` 区块会被自动移除；如仍装有 `robotpkg-*` 的 Pinocchio、
coal、eigenpy 包，脚本会提示先卸载，避免与源码安装混在同一前缀中。

### 安装其余工作区依赖

运行 rosdep 时跳过通用的 `pinocchio` key，避免再安装 `ros-jazzy-pinocchio`
而形成两套库：

```bash
cd ~/asr_sdm_robo
rosdep install \
  --from-paths src \
  --ignore-src \
  -r -y \
  --skip-keys pinocchio
```

如果 rosdep 尚未初始化：

```bash
sudo rosdep init
rosdep update
```

## 3. 验证安装

检查 C++ 包：

```bash
pkg-config --modversion pinocchio
```

检查 Python 绑定：

```bash
python3 -c "import pinocchio; print(pinocchio.__version__)"
```

构建后也可以确认 CMake 实际找到的位置：

```bash
grep pinocchio_DIR build/asr_sdm_kinematic_dynamic_model/CMakeCache.txt
```

预期路径位于：

```text
/opt/openrobots/lib/cmake/pinocchio
```

## 4. 构建和验证模型 package

模型检查程序仅在 `BUILD_TESTING=ON` 时生成，并依赖
`asr_sdm_head_following_control`。使用 `--packages-up-to` 可同时构建所需工作区依赖：

```bash
cd ~/asr_sdm_robo
source /opt/ros/jazzy/setup.bash
source ~/.bashrc

CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
  --packages-up-to asr_sdm_kinematic_dynamic_model \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON

source install/setup.bash
ros2 run asr_sdm_kinematic_dynamic_model asr_sdm_kinematic_model_check
```

使用单编译任务是为了降低 `pinocchio_dynamics_node.cpp` 编译时的内存占用。

## 5. coal 与 hpp-fcl 提示

hpp-fcl 已更名为 coal。旧版 Pinocchio 或兼容配置可能显示：

```text
Please update your CMake from 'hpp-fcl' to 'coal'
Please update your includes from 'hpp/fcl' to 'coal'
```

本工作区的模型源码没有直接使用 hpp-fcl；这类提示通常来自已安装 Pinocchio 的
传递依赖或兼容头文件。如果构建成功，不需要修改本工作区源码，也不要为了消除提示
而混装另一套 hpp-fcl。需要彻底消除提示时，应整体升级相互兼容的 Pinocchio 与 coal。

## 6. 避免混用两套安装

不要在同一次构建中混用：

```text
/opt/openrobots                    setup-dev-env.sh 源码安装（或旧的 robotpkg 包）
/opt/ros/jazzy                     ros-jazzy-pinocchio / ros-jazzy-coal
```

混用可能导致：

- CMake 找到一套头文件，却链接另一套库；
- Pinocchio、coal 或 eigenpy 版本不匹配；
- ABI、模板定义或 Python 绑定冲突。

排查时检查：

```bash
printenv CMAKE_PREFIX_PATH
printenv LD_LIBRARY_PATH
pkg-config --variable=prefix pinocchio
grep pinocchio_DIR build/asr_sdm_kinematic_dynamic_model/CMakeCache.txt
```

切换安装方式后，应清理相关 package 的 `build/` 和 `install/` 子目录再重新构建。

## 7. 常见问题

1. **找不到 Pinocchio（`pinocchio_DIR-NOTFOUND`）**：重新运行 `./setup-dev-env.sh --common`
   （已安装的组件会跳过，只补登记 CMake 注册表），然后重新 `colcon build`；也可以
   `source ~/.bashrc` 后再编译。
2. **Python 无法 import**：确认 `PYTHONPATH` 中的 Python 版本目录与
   `python3 --version` 一致。
3. **编译器进程被系统终止**：使用 `CMAKE_BUILD_PARALLEL_LEVEL=1` 和
   `--parallel-workers 1` 降低内存占用。
4. **网络或证书错误**：检查代理、DNS 和系统时间，然后重新运行 `./setup-dev-env.sh --common`。
   未完成的克隆目录（无 `CMakeLists.txt`）需先手动删除。
5. **没有 sudo 权限**：将 `PINOCCHIO_INSTALL_PREFIX` 改为用户可写目录后加 `--rebuild` 运行，
   脚本会把该前缀写入 `~/.bashrc`。
