# Pinocchio 安装说明

本文说明 ASR-SDM 工作区使用的 Pinocchio 安装方式。Pinocchio 项目地址：
[stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)。

推荐使用仓库根目录的 `setup-dev-env.sh --dependency`，通过 robotpkg 安装
Pinocchio、coal 和 eigenpy（该选项同时会安装 c-periphery 静态库）。需要特定
分支或本地修改时，再使用源码安装脚本。

## 1. 支持环境

推荐环境：

- Ubuntu 24.04 amd64；
- ROS 2 Jazzy；
- Python 3；
- 工作区位于 `~/asr_sdm_robo`，或将下列命令中的路径替换为实际路径。

robotpkg 安装流程目前使用 amd64 软件源，不适用于 ARM64。ARM64 平台应使用
适配该平台的发行版包或源码安装方式。

## 2. robotpkg 安装（推荐）

仓库脚本会：

- 配置 robotpkg apt 软件源及签名密钥；
- 根据当前 Python 版本安装对应绑定；
- 安装 Pinocchio、coal 和 eigenpy；
- 将 `/opt/openrobots` 相关环境变量写入 `~/.bashrc`。

安装 Pinocchio 工具链（`--dependency` 会同时安装 c-periphery）：

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --dependency
```

如果 ROS 2 Jazzy 尚未安装，可以一并安装：

```bash
cd ~/asr_sdm_robo
./setup-dev-env.sh --ros --dependency
```

脚本的 `-y` 选项仅用于 CI，不建议普通交互式安装使用。

安装完成后加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
```

脚本配置的主要环境变量为：

```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python<当前版本>/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

实际 Python 路径由安装脚本根据当前解释器版本生成，不要硬编码为 `py312`。

### 安装其余工作区依赖

采用 robotpkg Pinocchio 后，运行 rosdep 时跳过通用的 `pinocchio` key，避免再安装
`ros-jazzy-pinocchio` 而形成两套库：

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

## 3. 验证 robotpkg 安装

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

使用推荐的 robotpkg 环境时，预期路径位于：

```text
/opt/openrobots/lib/cmake/pinocchio
```

## 4. 源码安装（高级用法）

需要从上游稳定 release 构建 Pinocchio，或使用独立安装前缀时，可运行：

```bash
cd ~/asr_sdm_robo
python3 install_pinocchio_from_source.py
source ~/.bashrc
```

脚本不会固定版本，也不再默认跟随 `devel`。每次运行时会读取 Pinocchio、
eigenpy 和 Coal 的上游稳定语义版本标签，解析 release 中的兼容约束，并选择
当前最新的兼容组合。alpha、beta 和 release candidate 等预发布标签不会被选中。
在安装 apt 依赖或修改源码目录前，脚本会打印选中的 tag、commit SHA 和约束。

可以先执行无副作用的解析检查：

```bash
python3 install_pinocchio_from_source.py --resolve-only
```

该模式只访问上游仓库并显示选择结果，不运行 apt、不 checkout 源码，也不安装文件。
构建依赖已经准备好时，可使用 `--skip-dependencies` 跳过 apt 安装。

常用设置：

| 环境变量 | 默认值 | 说明 |
|---|---|---|
| `PINOCCHIO_REF` | 自动选择 | Pinocchio tag、branch 或 commit |
| `EIGENPY_REF` | 自动选择 | eigenpy tag、branch 或 commit |
| `COAL_REF` | 自动选择 | Coal tag、branch 或 commit |
| `PINOCCHIO_SRC_DIR` | `~/src/pinocchio` | Pinocchio 源码目录 |
| `EIGENPY_SRC_DIR` | `~/src/eigenpy` | eigenpy 源码目录 |
| `COAL_SRC_DIR` | `~/src/coal` | Coal 源码目录 |
| `PINOCCHIO_BUILD_DIR` | 按版本生成 | Pinocchio build 目录覆盖 |
| `EIGENPY_BUILD_DIR` | 按版本生成 | eigenpy build 目录覆盖 |
| `COAL_BUILD_DIR` | 按版本生成 | Coal build 目录覆盖 |
| `PINOCCHIO_INSTALL_PREFIX` | `/opt/openrobots` | 安装前缀 |
| `PINOCCHIO_BUILD_COLLISION` | `OFF` | 是否启用 Pinocchio 的碰撞支持 |

旧的 `PINOCCHIO_BRANCH` 和 `EIGENPY_BRANCH` 仍可作为兼容别名，但会打印弃用提示。
对于稳定 tag，脚本会验证其版本约束；显式 branch 或 commit 没有稳定语义版本时，
最终兼容性由 CMake configure/build 验证。

源码构建顺序固定为 eigenpy、Coal、Pinocchio。Coal 会由脚本自动选择和构建，
无需在启用碰撞支持前手工准备另一套 Coal。`PINOCCHIO_BUILD_COLLISION=ON`
只控制 Pinocchio 是否启用其碰撞接口：

```bash
export PINOCCHIO_BUILD_COLLISION=ON
python3 install_pinocchio_from_source.py
```

三个项目均采用 Release 构建，并关闭各自的测试、示例或文档构建。并行度统一为
检测到的逻辑 CPU 数量的一半（向下取整，最少 1 个 job）。后构建的组件会通过
`CMAKE_PREFIX_PATH` 显式使用本轮刚安装到 `PINOCCHIO_INSTALL_PREFIX` 的依赖。
用户可写的安装前缀会直接执行 `cmake --install`；只有受保护的系统前缀才使用
`sudo`。

## 5. 构建和验证模型 package

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

## 6. coal 与 hpp-fcl 提示

hpp-fcl 已更名为 coal。旧版 Pinocchio 或兼容配置可能显示：

```text
Please update your CMake from 'hpp-fcl' to 'coal'
Please update your includes from 'hpp/fcl' to 'coal'
```

本工作区的模型源码没有直接使用 hpp-fcl；这类提示通常来自已安装 Pinocchio 的
传递依赖或兼容头文件。如果构建成功，不需要修改本工作区源码，也不要为了消除提示
而混装另一套 hpp-fcl。需要彻底消除提示时，应整体升级相互兼容的 Pinocchio 与 coal。

## 7. 避免混用两套安装

不要在同一次构建中混用：

```text
/opt/openrobots                    robotpkg 或源码安装
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

## 8. 常见问题

1. **找不到 Pinocchio**：确认已加载 `~/.bashrc`，并检查
   `/opt/openrobots` 是否位于 `CMAKE_PREFIX_PATH`。
2. **Python 无法 import**：确认 `PYTHONPATH` 中的 Python 版本目录与
   `python3 --version` 一致。
3. **编译器进程被系统终止**：使用 `CMAKE_BUILD_PARALLEL_LEVEL=1` 和
   `--parallel-workers 1` 降低内存占用。
4. **网络或证书错误**：检查代理、DNS 和系统时间，然后重新运行安装脚本。
5. **没有 sudo 权限**：源码安装时将 `PINOCCHIO_INSTALL_PREFIX` 改为用户可写目录，
   并相应设置环境变量。
