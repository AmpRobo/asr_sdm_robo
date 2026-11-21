# [WIP] asr_sdm_robo
Workspace for Amphibious Snake-like Robot with Screw-drive Mechanism

**English** | [中文](#中文)

## Remote PC Setup

### Dependency Installation

`setup-dev-env.sh` installs the development dependencies. It targets **Ubuntu 24.04 (noble)**, and the NVIDIA role additionally requires **x86_64**. Each role is opt-in, and re-running the script is safe: finished steps are detected and skipped. Run it as your normal user; it calls `sudo` itself where needed.

```sh
# Show every installable component with its versions
./setup-dev-env.sh --list

# Install a single role
./setup-dev-env.sh --common
./setup-dev-env.sh --ros
./setup-dev-env.sh --nvidia

# Full setup without prompts (intended for CI)
./setup-dev-env.sh --ros --common --nvidia -y

# Force the source-built libraries (Slint, OpenCV) to be rebuilt
./setup-dev-env.sh --common --nvidia --rebuild
```

| Flag | Installs |
|---|---|
| `--common` | Build toolchain (`build-essential`, `cmake`, `ninja-build`, `ccache`, `gdb`, `clang-format`), `git` / `git-lfs`, Python 3 dev packages, Qt 6 + QML modules, Coral GUI libraries (OpenSceneGraph, Vulkan, Assimp), PCL, glog, `xacro`, everyday CLI tools, Rust stable (>= 1.95, via rustup), and these libraries built from source: [Slint](https://slint.dev) 1.19.0 C++ and [Matplot++](https://github.com/alandefreitas/matplotplusplus) 1.2.2 into `/usr/local`, [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 4.1.0 + eigenpy 3.13.0 + Coal 3.0.4 (with Python bindings) into `/opt/openrobots`, and the [c-periphery](https://github.com/AmpRobo/c-periphery) 2.5.0 static library into `/usr/local` |
| `--ros` | ROS 2 Jazzy desktop, `rosdep`, `colcon`, and the matching `~/.bashrc` entries |
| `--nvidia` | NVIDIA driver 595, CUDA Toolkit 13.2, cuBLAS / NPP / cuFFT, cuDNN 9, TensorRT 11.1.0, and OpenCV 4.14.0 built with CUDA |

Modifiers:

| Flag | Effect |
|---|---|
| `-y` | Non-interactive mode (skips the confirmation prompt) |
| `--no-nvidia` | Skip the whole NVIDIA role |
| `--no-cuda-drivers` | Install the NVIDIA role but keep the current driver |
| `--runtime` | Install runtime packages only, without the CUDA and TensorRT dev packages |
| `--rebuild` | Rebuild Slint, Matplot++, the Pinocchio stack, c-periphery and OpenCV even when already installed |

Notes on the `--common` role:

- `xacro` comes from the ROS apt repository, so run `--ros` before (or together with) `--common`; otherwise it is skipped with a warning.
- Rust is installed per user with rustup into `~/.cargo` and `~/.rustup`, and rustup adds `. "$HOME/.cargo/env"` to `~/.bashrc`. An existing toolchain older than 1.95 is updated, and `stable` becomes the default.
- Slint is built once (several minutes) and skipped afterwards while `/usr/local/bin/slint-compiler` reports 1.19.0. Manual steps: [`docs/slint_setup.md`](docs/slint_setup.md).
- Pinocchio, eigenpy and Coal are pinned to a mutually compatible set and built in that order with half of the CPUs, which keeps the memory use of the Python bindings in check. `~/.bashrc` gets the `/opt/openrobots` exports. Set `PINOCCHIO_BUILD_COLLISION=ON` (with `--rebuild`) for collision support; see [`docs/pinocchio_installation.md`](docs/pinocchio_installation.md). Remove any old `robotpkg-*` Pinocchio packages first; the script warns when it finds them.
- Each source build leaves a stamp in its `build/` directory, so later runs skip it while the library is still installed.

Notes on the `--nvidia` role:

- The driver is only installed when the current one is older than branch 595, so an up-to-date machine is left untouched. **Reboot** after a driver install before `nvidia-smi` works.
- TensorRT is not publicly downloadable. Register the NVIDIA local apt repo first (`nv-tensorrt-local-repo-ubuntu2404-*.deb` from [NVIDIA TensorRT](https://developer.nvidia.com/tensorrt)); otherwise the script prints instructions and skips that step.
- OpenCV is built from source into `~/.local/opencv-cuda`, which takes a while. Later runs skip the build while 4.14.0 with the CUDA modules is installed there (use `--rebuild` to force it). Ubuntu's `libopencv-dev` has no CUDA modules and cannot replace it.
- The resulting `CUDA_HOME`, `OpenCV_DIR`, `LD_LIBRARY_PATH` and `PKG_CONFIG_PATH` exports are written to `~/.bashrc`. Run `source ~/.bashrc` before `colcon build`, or source `~/.local/opencv-cuda/setup_env.sh` for a single shell.

See [`src/asr_sdm_universe/common/tensorrt_common/README.md`](src/asr_sdm_universe/common/tensorrt_common/README.md) for the tested version matrix and the equivalent manual steps.

#### The `dependency/` folder

Everything the script downloads or compiles is kept in a `dependency/` folder next to the repository (`../dependency`, beside `asr_sdm_robo/`), so it stays out of the git tree and the colcon workspace. Set `ASR_SDM_DEPENDENCY_DIR` to use another location. The folder carries a `COLCON_IGNORE` marker and is handed back to your user even when the script runs under `sudo`. A `dependency/` left inside the repository by an older version of the script is moved there on the next run, keeping finished builds.

| Path | Contents |
|---|---|
| `slint-1.19.0/`, `slint-1.19.0.zip` | Slint source and its `build/` directory |
| `matplotplusplus-1.2.2/` | Matplot++ clone and its `build/` directory |
| `eigenpy-3.13.0/`, `coal-3.0.4/`, `pinocchio-4.1.0/` | Pinocchio stack clones (with the `cmake/` submodule) and their `build/` directories |
| `opencv-4.14.0/`, `opencv_contrib-4.14.0/` (+ `.zip`) | OpenCV sources; the CUDA build runs in `opencv-4.14.0/build/` |
| `c-periphery/` | c-periphery clone and its `build/` directory |
| `cargo/` | Crates downloaded by cargo while building Slint |
| `rustup-init.sh` | rustup installer |
| `ros2-apt-source_*.deb`, `cuda-keyring_*.deb` | apt repository packages |

Downloads are reused on the next run, so an offline machine only needs these files copied in. Only installed results live elsewhere: apt packages, `/usr/local` (Slint, Matplot++, c-periphery), `/opt/openrobots` (Pinocchio, eigenpy, Coal), `~/.local/opencv-cuda`, and the Rust toolchain in `~/.rustup`. Set `OPENCV_CUDA_BUILD_ROOT` to build OpenCV somewhere else.

### ROS Workspace Installation

Clone the repository. It is a colcon workspace, so its root is used directly as the workspace root.

```sh
git clone https://github.com/AmpRobo/asr_sdm_robo.git
cd asr_sdm_robo
```

Resolve the package dependencies declared in each `package.xml`:

```sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace. `--parallel-workers` keeps the memory usage of the heavier perception packages in check:

```sh
colcon build --symlink-install --parallel-workers 2
```

Then overlay the result onto the ROS environment. Repeat this in every new shell, or append it to `~/.bashrc`:

```sh
source install/setup.bash
```

Partial builds are available for iterating on a subset of the workspace:

```sh
# Skip every package under src/asr_sdm_universe/perception/
./build_without_perception.sh

# Build only the VINS packages
./build_video_inertial_navigation_systems.sh

# Preview the affected packages without building
./build_without_perception.sh --list-only

# Build a single package
colcon build --symlink-install --packages-select asr_sdm_monitor
```

Notes:

- Run `./setup-dev-env.sh` (see above) before the first build. The perception packages additionally need the `--nvidia` role, so `source ~/.bashrc` first to pick up `CUDA_HOME` and `OpenCV_DIR`.
- `rosdep` only resolves the keys declared in `package.xml`. CUDA, cuDNN, TensorRT and CUDA-enabled OpenCV are not covered and must come from `--nvidia`. Pinocchio, Matplot++ and c-periphery come from `--common` (pass `--skip-keys pinocchio` to `rosdep install`).
- To rebuild from scratch, remove the generated directories: `rm -rf build install log`.

### asr_sdm_monitor

The monitor UI uses Slint and Qt 6, both installed by `./setup-dev-env.sh --common` (Slint into `/usr/local`, found by `find_package(Slint)` without extra arguments). See [`docs/slint_setup.md`](docs/slint_setup.md) to install Slint by hand.

```sh
# Qt6+ QML Dependency (already covered by ./setup-dev-env.sh --common)
sudo apt update
sudo apt install -y qt6-base-dev qt6-declarative-dev qt6-tools-dev qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts qml6-module-qtqml-workerscript qml6-module-qtquick-window qml6-module-qtqml qml6-module-qtquick-templates

# Run monitor
ros2 run asr_sdm_monitor asr_sdm_monitor
```


## Radxa Zero Setup

### .bash.rc
```sh
alias ls='ls --color=auto'  # Linux
alias ls='ls -G'            # macOS
export CLICOLOR=1           # macOS 启用颜色
export LSCOLORS="ExGxFxdaCxDaDahbadacec"  # macOS 颜色方案
export TERM=xterm-256color
export PS1='\[\e[1;32m\]\u@\h:\w\$\[\e[0m\] '  # 绿色提示符
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=176
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/2/
```

### C Library for Linux Peripheral I/O (GPIO, LED, PWM, SPI, I2C, MMIO, Serial)
https://github.com/AmpRobo/c-periphery

### How to grant user permissions to utilize the spidev port
```sh
ls -l /dev/spidev*
sudo groupadd spi
sudo usermod -aG spi $USER
sudo nano /etc/udev/rules.d/99-spi.rules
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"

sudo groupadd gpio
sudo usermod -aG gpio $USER
sudo nano /etc/udev/rules.d/99-gpio.rules
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"

# Check the uart group
ls -l /dev/ttyS*
# You’ll see something like:
crw-rw---- 1 root dialout 4, 64 Sep  4 00:12 /dev/ttyS*
# In this example, the group is dialout. Suppose the group is dialout, run:
sudo usermod -aG dialout $USER
# Reboot ,then check with:
cat /dev/ttyS*

sudo nano /etc/udev/rules.d/99-uart.rules
SUBSYSTEM=="tty", GROUP="uart", MODE="0660"
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

### Add .dtbo filesm
```sh
sudo nano /etc/default/u-boot

# uncomment and modify the following lines
U_BOOT_UPDATE="true"
U_BOOT_FDT_OVERLAYS="rk3568-spi3-m1-cs0-mcp2515.dtbo rk3568-i2c4-m0.dtbo"
U_BOOT_FDT_OVERLAYS_DIR="/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/overlay"
U_BOOT_SYNC_DTBS="true"

# update u-boot
sudo u-boot-update
```

### Driver installation
Driver files are all saved here.
```sh
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/spi/mcp251x.ko
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
sudo insmod /lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
```

Add the drivers to automatic startup list.
```sh
sudo nano /etc/modules-load.d/modules-load.d
# add the following two lines
can-dev
mcp251x.ko
```

Check if the driver is installed successfully.
```sh
lsmod | grep spidev
dmesg | grep can
dmesg | grep mcp251
```

### Interface test
```sh
sudo ifconfig can0 txqueuelen 65536
sudo ip link set can0 up type can bitrate 500000
candump can0
ip link show
ip -details -statistics link show can0
```

## Dependencies

### glog
Install the official glog library. This is used instead of the `glog_vendor` package. Already covered by `./setup-dev-env.sh --common`.
```sh
sudo apt-get update && sudo apt-get install -y libgoogle-glog-dev
```

## ROS2

### Source code compilation
```sh
colcon build --symlink-install --parallel-workers 2
```

# Contributing

1. Fork it (<https://github.com/AmpRobo/asr_sdm_robo/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -asm 'feat: add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

---

# 中文

[English](#wip-asr_sdm_robo) | **中文**

螺旋驱动两栖蛇形机器人（Amphibious Snake-like Robot with Screw-drive Mechanism）的工作空间。

## 远程 PC 配置

### 依赖安装

`setup-dev-env.sh` 用于安装开发依赖。目标环境为 **Ubuntu 24.04（noble）**，其中 NVIDIA 部分还要求 **x86_64** 架构。各类别均需显式指定，脚本可重复执行：已完成的步骤会被检测并跳过。请以普通用户运行，脚本会在需要时自行调用 `sudo`。

```sh
# 列出所有可安装组件及其版本
./setup-dev-env.sh --list

# 单独安装某一类别
./setup-dev-env.sh --common
./setup-dev-env.sh --ros
./setup-dev-env.sh --nvidia

# 全部安装且不询问（供 CI 使用）
./setup-dev-env.sh --ros --common --nvidia -y

# 强制重新编译源码安装的库（Slint、OpenCV）
./setup-dev-env.sh --common --nvidia --rebuild
```

| 选项 | 安装内容 |
|---|---|
| `--common` | 编译工具链（`build-essential`、`cmake`、`ninja-build`、`ccache`、`gdb`、`clang-format`）、`git` / `git-lfs`、Python 3 开发包、Qt 6 与 QML 模块、Coral GUI 依赖库（OpenSceneGraph、Vulkan、Assimp）、PCL、glog、`xacro`、常用命令行工具、Rust stable（>= 1.95，通过 rustup），以及从源码编译的库：安装到 `/usr/local` 的 [Slint](https://slint.dev) 1.19.0 C++ 与 [Matplot++](https://github.com/alandefreitas/matplotplusplus) 1.2.2，安装到 `/opt/openrobots` 的 [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 4.1.0 + eigenpy 3.13.0 + Coal 3.0.4（含 Python 绑定），以及安装到 `/usr/local` 的 [c-periphery](https://github.com/AmpRobo/c-periphery) 2.5.0 静态库 |
| `--ros` | ROS 2 Jazzy desktop、`rosdep`、`colcon`，并写入对应的 `~/.bashrc` 配置 |
| `--nvidia` | NVIDIA 驱动 595、CUDA Toolkit 13.2、cuBLAS / NPP / cuFFT、cuDNN 9、TensorRT 11.1.0，以及启用 CUDA 编译的 OpenCV 4.14.0 |

修饰选项：

| 选项 | 作用 |
|---|---|
| `-y` | 非交互模式（跳过确认提示） |
| `--no-nvidia` | 跳过整个 NVIDIA 类别 |
| `--no-cuda-drivers` | 安装 NVIDIA 类别但保留当前驱动 |
| `--runtime` | 仅安装运行时包，不安装 CUDA 与 TensorRT 的开发包 |
| `--rebuild` | 即使已安装，也重新编译 Slint、Matplot++、Pinocchio 组件、c-periphery 与 OpenCV |

关于 `--common` 的说明：

- `xacro` 来自 ROS apt 源，请在 `--common` 之前（或同时）执行 `--ros`；否则会打印警告并跳过。
- Rust 通过 rustup 按用户安装到 `~/.cargo` 与 `~/.rustup`，rustup 会把 `. "$HOME/.cargo/env"` 写入 `~/.bashrc`。已有工具链低于 1.95 时会被更新，并把 `stable` 设为默认。
- Slint 只编译一次（需要几分钟），之后只要 `/usr/local/bin/slint-compiler` 报告 1.19.0 就会跳过。手动步骤见 [`docs/slint_setup.md`](docs/slint_setup.md)。
- Pinocchio、eigenpy、Coal 固定为一组相互兼容的版本，按此顺序以一半 CPU 并行编译，以控制 Python 绑定的内存占用；`/opt/openrobots` 相关环境变量写入 `~/.bashrc`。需要碰撞支持时设置 `PINOCCHIO_BUILD_COLLISION=ON` 并加 `--rebuild`，详见 [`docs/pinocchio_installation.md`](docs/pinocchio_installation.md)。请先卸载旧的 `robotpkg-*` Pinocchio 包，脚本检测到时会提示。
- 每个源码编译项目会在其 `build/` 中留下标记文件，之后只要库仍已安装就会跳过。

关于 `--nvidia` 的说明：

- 仅当现有驱动低于 595 分支时才会安装驱动，已达标的机器不会被改动。安装驱动后需**重启**，`nvidia-smi` 才可用。
- TensorRT 无法公开下载，需先注册 NVIDIA 本地 apt 源（从 [NVIDIA TensorRT](https://developer.nvidia.com/tensorrt) 获取 `nv-tensorrt-local-repo-ubuntu2404-*.deb`）；否则脚本会打印操作指引并跳过该步骤。
- OpenCV 从源码编译安装到 `~/.local/opencv-cuda`，耗时较长。之后只要该目录中已有带 CUDA 模块的 4.14.0 就会跳过编译（用 `--rebuild` 强制重编）。Ubuntu 自带的 `libopencv-dev` 不含 CUDA 模块，无法替代。
- 生成的 `CUDA_HOME`、`OpenCV_DIR`、`LD_LIBRARY_PATH`、`PKG_CONFIG_PATH` 会写入 `~/.bashrc`。执行 `colcon build` 前请先 `source ~/.bashrc`，或仅在当前终端 source `~/.local/opencv-cuda/setup_env.sh`。

已验证的版本矩阵与等效的手动安装步骤见 [`src/asr_sdm_universe/common/tensorrt_common/README.md`](src/asr_sdm_universe/common/tensorrt_common/README.md)。

#### `dependency/` 目录

脚本下载或编译的所有内容都放在与仓库并列的 `dependency/` 目录中（`../dependency`，与 `asr_sdm_robo/` 同级），不进入 git 仓库和 colcon 工作空间。可通过环境变量 `ASR_SDM_DEPENDENCY_DIR` 指定其他位置。该目录带有 `COLCON_IGNORE` 标记；即使脚本在 `sudo` 下运行，目录所有者也会改回当前用户。旧版脚本留在仓库内的 `dependency/` 会在下次运行时自动移过去，已完成的编译会保留。

| 路径 | 内容 |
|---|---|
| `slint-1.19.0/`、`slint-1.19.0.zip` | Slint 源码及其 `build/` 目录 |
| `matplotplusplus-1.2.2/` | Matplot++ 克隆及其 `build/` 目录 |
| `eigenpy-3.13.0/`、`coal-3.0.4/`、`pinocchio-4.1.0/` | Pinocchio 组件克隆（含 `cmake/` 子模块）及其 `build/` 目录 |
| `opencv-4.14.0/`、`opencv_contrib-4.14.0/`（及 `.zip`） | OpenCV 源码；CUDA 版在 `opencv-4.14.0/build/` 中编译 |
| `c-periphery/` | c-periphery 克隆及其 `build/` 目录 |
| `cargo/` | 编译 Slint 时 cargo 下载的 crate |
| `rustup-init.sh` | rustup 安装脚本 |
| `ros2-apt-source_*.deb`、`cuda-keyring_*.deb` | apt 源安装包 |

已下载的文件在下次运行时会直接复用，离线机器只需拷入这些文件。只有安装结果位于其他位置：apt 包、`/usr/local`（Slint、Matplot++、c-periphery）、`/opt/openrobots`（Pinocchio、eigenpy、Coal）、`~/.local/opencv-cuda`，以及 `~/.rustup` 中的 Rust 工具链。设置 `OPENCV_CUDA_BUILD_ROOT` 可把 OpenCV 编译目录放到别处。

### ROS 工作空间安装

克隆仓库。本仓库本身即为 colcon 工作空间，其根目录直接作为工作空间根目录使用。

```sh
git clone https://github.com/AmpRobo/asr_sdm_robo.git
cd asr_sdm_robo
```

解析各 `package.xml` 中声明的依赖：

```sh
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

编译工作空间。`--parallel-workers` 用于控制较重的 perception 包的内存占用：

```sh
colcon build --symlink-install --parallel-workers 2
```

随后将编译结果叠加到 ROS 环境中。每开一个新终端都需执行，或写入 `~/.bashrc`：

```sh
source install/setup.bash
```

若只需迭代部分包，可使用以下局部编译方式：

```sh
# 跳过 src/asr_sdm_universe/perception/ 下的所有包
./build_without_perception.sh

# 仅编译 VINS 相关包
./build_video_inertial_navigation_systems.sh

# 仅预览受影响的包，不执行编译
./build_without_perception.sh --list-only

# 仅编译单个包
colcon build --symlink-install --packages-select asr_sdm_monitor
```

说明：

- 首次编译前请先执行 `./setup-dev-env.sh`（见上文）。perception 相关包还需要 `--nvidia` 类别，编译前先 `source ~/.bashrc` 以加载 `CUDA_HOME` 与 `OpenCV_DIR`。
- `rosdep` 只能解析 `package.xml` 中声明的依赖键，CUDA、cuDNN、TensorRT 以及启用 CUDA 的 OpenCV 不在其覆盖范围内，需通过 `--nvidia` 安装。Pinocchio、Matplot++ 与 c-periphery 均由 `--common` 安装（`rosdep install` 时加 `--skip-keys pinocchio`）。
- 需要完全重新编译时，删除生成目录即可：`rm -rf build install log`。

### asr_sdm_monitor

monitor 界面使用 Slint 与 Qt 6，二者都由 `./setup-dev-env.sh --common` 安装（Slint 装在 `/usr/local`，`find_package(Slint)` 无需额外参数即可找到）。手动安装 Slint 见 [`docs/slint_setup.md`](docs/slint_setup.md)。

```sh
# Qt6+ QML 依赖（./setup-dev-env.sh --common 已包含）
sudo apt update
sudo apt install -y qt6-base-dev qt6-declarative-dev qt6-tools-dev qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts qml6-module-qtqml-workerscript qml6-module-qtquick-window qml6-module-qtqml qml6-module-qtquick-templates

# 启动 monitor
ros2 run asr_sdm_monitor asr_sdm_monitor
```

## Radxa Zero 配置

### .bash.rc
```sh
alias ls='ls --color=auto'  # Linux
alias ls='ls -G'            # macOS
export CLICOLOR=1           # macOS 启用颜色
export LSCOLORS="ExGxFxdaCxDaDahbadacec"  # macOS 颜色方案
export TERM=xterm-256color
export PS1='\[\e[1;32m\]\u@\h:\w\$\[\e[0m\] '  # 绿色提示符
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=176
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/2/
```

### Linux 外设 I/O C 库（GPIO、LED、PWM、SPI、I2C、MMIO、串口）
https://github.com/AmpRobo/c-periphery

### 如何为用户授予 spidev 端口权限
```sh
ls -l /dev/spidev*
sudo groupadd spi
sudo usermod -aG spi $USER
sudo nano /etc/udev/rules.d/99-spi.rules
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"

sudo groupadd gpio
sudo usermod -aG gpio $USER
sudo nano /etc/udev/rules.d/99-gpio.rules
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"

# 查看 uart 所属用户组
ls -l /dev/ttyS*
# 输出类似：
crw-rw---- 1 root dialout 4, 64 Sep  4 00:12 /dev/ttyS*
# 本例中用户组为 dialout，若同为 dialout 则执行：
sudo usermod -aG dialout $USER
# 重启后用以下命令确认：
cat /dev/ttyS*

sudo nano /etc/udev/rules.d/99-uart.rules
SUBSYSTEM=="tty", GROUP="uart", MODE="0660"
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

### 添加 .dtbo 文件
```sh
sudo nano /etc/default/u-boot

# 取消注释并修改以下几行
U_BOOT_UPDATE="true"
U_BOOT_FDT_OVERLAYS="rk3568-spi3-m1-cs0-mcp2515.dtbo rk3568-i2c4-m0.dtbo"
U_BOOT_FDT_OVERLAYS_DIR="/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/overlay"
U_BOOT_SYNC_DTBS="true"

# 更新 u-boot
sudo u-boot-update
```

### 驱动安装
驱动文件均位于以下位置。
```sh
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/spi/mcp251x.ko
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
sudo insmod /lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
```

将驱动加入开机自动加载列表。
```sh
sudo nano /etc/modules-load.d/modules-load.d
# 添加以下两行
can-dev
mcp251x.ko
```

确认驱动是否安装成功。
```sh
lsmod | grep spidev
dmesg | grep can
dmesg | grep mcp251
```

### 接口测试
```sh
sudo ifconfig can0 txqueuelen 65536
sudo ip link set can0 up type can bitrate 500000
candump can0
ip link show
ip -details -statistics link show can0
```

## 依赖库

### glog
安装官方 glog 库，用以替代 `glog_vendor` 包。`./setup-dev-env.sh --common` 已包含。
```sh
sudo apt-get update && sudo apt-get install -y libgoogle-glog-dev
```

## ROS2

### 源码编译
```sh
colcon build --symlink-install --parallel-workers 2
```

## 贡献方式

1. Fork 本仓库（<https://github.com/AmpRobo/asr_sdm_robo/fork>）
2. 创建特性分支（`git checkout -b feature/fooBar`）
3. 提交改动（`git commit -asm 'feat: add some fooBar'`）
4. 推送分支（`git push origin feature/fooBar`）
5. 创建 Pull Request

