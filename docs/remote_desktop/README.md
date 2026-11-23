# GNOME Remote Desktop: rviz2 crashes the remote session

**English** | [中文](#中文)

## Symptom

In a GNOME Remote Desktop (RDP) **remote login** session, opening rviz2 (for example through `ros2 launch planning_simulator planning_simulator.launch.py`) closes the whole remote desktop at once. Every window in the session is gone, and so is any unsaved work.

The journal shows the same two lines every time, in the same second the GL window opens:

```text
gnome-shell[...]: Window manager warning: Invalid WM_TRANSIENT_FOR window 0x...08 specified for 0x...06.
gnome-shell[...]: GNOME Shell crashed with signal 11
```

Check with:

```sh
journalctl -b | grep -E 'GNOME Shell crashed|WM_TRANSIENT_FOR'
```

## Cause

- A remote-login session has no seat, so gnome-shell (mutter 46) runs **headless** with a surfaceless EGL renderer.
- glvnd lists NVIDIA's EGL vendor (`10_nvidia.json`) before Mesa's, so the headless shell renders through NVIDIA EGL.
- rviz2 (Qt/X11) goes through Xwayland. When mutter handles that window's shared-memory buffer, it dereferences an invalid context in `meta_context_get_backend()` and segfaults. The ROS nodes and rviz2 are not at fault.
- The NVIDIA driver branch does not matter: the crash happens with both 595 and 615. Rendering rviz2 with Mesa (`LIBGL_ALWAYS_SOFTWARE=1`) does not help either, because the crash is in the compositor, not in the client.

Tested on Ubuntu 24.04, gnome-shell 46.0-0ubuntu6~24.04.17, mutter 46.2-1ubuntu0.24.04.18, NVIDIA 595.99.02 / 615.71.09 (open), RTX 3060.

## Workaround

Run the remote session's gnome-shell on **Mesa EGL** (llvmpipe). A systemd user drop-in starts gnome-shell through a small wrapper. The wrapper sets `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json` only when `loginctl` reports the session as `Remote=yes`.

| File | Installed to |
|------|--------------|
| [`gnome-shell-remote-egl`](gnome-shell-remote-egl) | `~/.local/bin/gnome-shell-remote-egl` |
| [`remote-mesa-egl.conf`](remote-mesa-egl.conf) | `~/.config/systemd/user/org.gnome.Shell@wayland.service.d/remote-mesa-egl.conf` |
| [`install.sh`](install.sh) | (installer, run from this folder) |

### Install

```sh
./docs/remote_desktop/install.sh
```

Then **log out** of the remote session (top-right menu > Power > Log Out) and connect again. Closing the RDP window is not enough: the session, and the old gnome-shell, keep running.

### Verify

```sh
# Expect __EGL_VENDOR_LIBRARY_FILENAMES=.../50_mesa.json and only libEGL_mesa (no libnvidia-eglcore)
P=$(pgrep -u "$USER" -x gnome-shell)
tr '\0' '\n' < /proc/$P/environ | grep __EGL
grep -oE '(libEGL_[a-z]+|libnvidia-eglcore)[^ ]*' /proc/$P/maps | sort -u
```

After that, rviz2 opens without crashing the session. The `WM_TRANSIENT_FOR` warning is still logged and is harmless.

### Uninstall

```sh
./docs/remote_desktop/install.sh --uninstall
```

Then log out and log in again.

## Side effects

- **Local logins are unchanged**: the variable is only set when the session is remote.
- In the remote session, the desktop is composited on the CPU.
- Xwayland and every program started in the remote session inherit the variable. X11 GL programs, rviz2 included, then render with llvmpipe (`OpenGL version: 4.5` in the rviz2 log instead of 4.6 on the NVIDIA GPU). Large point clouds can drop the frame rate.
- CUDA, TensorRT and `nvidia-smi` are not affected: they do not use EGL.

## If the remote login fails after installing

Log in over SSH and remove the drop-in, then connect again:

```sh
rm ~/.config/systemd/user/org.gnome.Shell@wayland.service.d/remote-mesa-egl.conf
```

---

## 中文

## 现象

在 GNOME 远程桌面（RDP）的**远程登录**会话中，一打开 rviz2（例如通过 `ros2 launch planning_simulator planning_simulator.launch.py`），整个远程桌面立刻闪退。会话里的所有窗口都会关闭，未保存的内容也会丢失。

每次在 GL 窗口打开的同一秒，journal 里都会出现这两行：

```text
gnome-shell[...]: Window manager warning: Invalid WM_TRANSIENT_FOR window 0x...08 specified for 0x...06.
gnome-shell[...]: GNOME Shell crashed with signal 11
```

检查方法：

```sh
journalctl -b | grep -E 'GNOME Shell crashed|WM_TRANSIENT_FOR'
```

## 原因

- 远程登录会话没有 seat，gnome-shell（mutter 46）以**无头模式**运行，使用 surfaceless EGL 渲染。
- glvnd 先列出 NVIDIA 的 EGL（`10_nvidia.json`），后列出 Mesa，所以无头 gnome-shell 通过 NVIDIA EGL 渲染。
- rviz2（Qt/X11）经由 Xwayland 显示。mutter 处理该窗口的共享内存缓冲区时，在 `meta_context_get_backend()` 中访问了无效的 context，导致段错误。问题不在 ROS 节点，也不在 rviz2。
- 与 NVIDIA 驱动分支无关：595 和 615 都会崩溃。让 rviz2 改用 Mesa 软件渲染（`LIBGL_ALWAYS_SOFTWARE=1`）也没用，因为崩溃发生在合成器里，而不是客户端。

测试环境：Ubuntu 24.04，gnome-shell 46.0-0ubuntu6~24.04.17，mutter 46.2-1ubuntu0.24.04.18，NVIDIA 595.99.02 / 615.71.09（open），RTX 3060。

## 解决办法

让远程会话中的 gnome-shell 改用 **Mesa EGL**（llvmpipe）。通过 systemd 用户级 drop-in，用一个包装脚本启动 gnome-shell。只有当 `loginctl` 报告会话为 `Remote=yes` 时，脚本才设置 `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json`。

| 文件 | 安装位置 |
|------|----------|
| [`gnome-shell-remote-egl`](gnome-shell-remote-egl) | `~/.local/bin/gnome-shell-remote-egl` |
| [`remote-mesa-egl.conf`](remote-mesa-egl.conf) | `~/.config/systemd/user/org.gnome.Shell@wayland.service.d/remote-mesa-egl.conf` |
| [`install.sh`](install.sh) | （安装脚本，在本目录中运行） |

### 安装

```sh
./docs/remote_desktop/install.sh
```

然后**注销**远程会话（右上角菜单 > 电源 > 注销），再重新连接。只关闭 RDP 窗口是不够的：会话和旧的 gnome-shell 会继续运行。

### 验证

```sh
# 应看到 __EGL_VENDOR_LIBRARY_FILENAMES=.../50_mesa.json，且只加载 libEGL_mesa（没有 libnvidia-eglcore）
P=$(pgrep -u "$USER" -x gnome-shell)
tr '\0' '\n' < /proc/$P/environ | grep __EGL
grep -oE '(libEGL_[a-z]+|libnvidia-eglcore)[^ ]*' /proc/$P/maps | sort -u
```

之后打开 rviz2 不会再让会话崩溃。日志里仍会出现 `WM_TRANSIENT_FOR` 警告，可以忽略。

### 卸载

```sh
./docs/remote_desktop/install.sh --uninstall
```

然后注销并重新登录。

## 副作用

- **本地登录不受影响**：只有远程会话才会设置该变量。
- 远程会话中，桌面合成由 CPU 完成。
- Xwayland 以及在远程会话中启动的所有程序都会继承该变量。包括 rviz2 在内的 X11 GL 程序会改用 llvmpipe 渲染（rviz2 日志显示 `OpenGL version: 4.5`，而用 NVIDIA 显卡时是 4.6）。点云较大时帧率可能下降。
- CUDA、TensorRT 和 `nvidia-smi` 不受影响，它们不使用 EGL。

## 安装后无法远程登录时

通过 SSH 登录并删除 drop-in，然后重新连接：

```sh
rm ~/.config/systemd/user/org.gnome.Shell@wayland.service.d/remote-mesa-egl.conf
```
