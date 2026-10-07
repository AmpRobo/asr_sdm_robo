# Slint C++ API 安装说明

本文说明如何从本地源码编译并安装 Slint C++ API。源码目录为
`/home/michael/Documents/code/dependency/slint`，CMake 工程版本为 1.19.0。

系统范围安装使用 `/usr/local`，需要 `sudo`。没有免密 sudo 时，可以安装到
`~/.local`。

## 1. 支持环境

- Ubuntu 24.04 amd64
- CMake 3.21 或更新（本机为 3.28.3）
- Ninja
- 支持 C++20 的编译器（本机为 GCC 13.3）
- Rust 1.95 或更新（本机通过 rustup 安装 stable 1.99.0）

## 2. 安装系统依赖

```bash
sudo apt install \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  libfontconfig-dev \
  libxkbcommon-dev \
  libxcb-shape0-dev \
  libxcb-xfixes0-dev \
  libssl-dev
```

`libxcb-shape0-dev` 和 `libxcb-xfixes0-dev` 是 Slint 文档列出的 X11 开发包。
本机当时只有对应的运行库，没有这两个开发包；Release 构建仍然完成。新机器按
上面的命令把开发包装齐。

## 3. 安装 Rust

Slint 的 C++ 库由 Rust 实现，编译时需要 `rustc` 和 `cargo`。已安装且版本不低于
1.95 时跳过本节。

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable
. "$HOME/.cargo/env"
rustc --version
```

rustup 会把 `$HOME/.cargo/env` 写入 `~/.bashrc`。新开的终端一般可以直接使用
`cargo`。当前终端若提示找不到 `cargo`，再执行一次 `. "$HOME/.cargo/env"`。

## 4. 编译

在源码根目录配置并编译 Release 版本。示例和测试默认关闭。

安装到 `/usr/local`：

```bash
cd /home/michael/Documents/code/dependency/slint
mkdir -p cppbuild && cd cppbuild
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
cmake --build .
```

安装到 `~/.local` 时，把上面的前缀换成 `"$HOME/.local"`：

```bash
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/.local" ..
cmake --build .
```

`cppbuild` 已经按 `CMAKE_INSTALL_PREFIX=$HOME/.local` 配置并完成 Release 编译时，
不必为了更换安装位置而重新编译。安装命令里的 `--prefix` 会覆盖配置时的前缀。

## 5. 安装

### 安装到 /usr/local

构建目录已经存在时：

```bash
cd /home/michael/Documents/code/dependency/slint/cppbuild
sudo cmake --install . --prefix /usr/local
```

若第 4 节配置时已经把 `CMAKE_INSTALL_PREFIX` 设为 `/usr/local`，安装命令可以写成：

```bash
sudo cmake --install .
```

安装结果：

- `/usr/local/lib/libslint_cpp.so`
- `/usr/local/include/slint/`
- `/usr/local/lib/cmake/Slint/`
- `/usr/local/bin/slint-compiler`

`/usr/local` 在 CMake 和 `PATH` 的默认搜索路径中，使用 `find_package(Slint)` 时不必再设置 `CMAKE_PREFIX_PATH`。

### 安装到 ~/.local

不需要 sudo：

```bash
cd /home/michael/Documents/code/dependency/slint/cppbuild
cmake --install . --prefix "$HOME/.local"
```

这一份已经安装过。对应文件为：

- `~/.local/lib/libslint_cpp.so`
- `~/.local/include/slint/`
- `~/.local/lib/cmake/Slint/`
- `~/.local/bin/slint-compiler`

`~/.local/bin` 已在当前 `PATH` 中。CMake 不会默认搜索 `~/.local`，链接该副本时需要：

```bash
-DCMAKE_PREFIX_PATH=$HOME/.local
```

两份同时存在时，CMake 使用 `CMAKE_PREFIX_PATH` 里排在前面的那一份。当前环境的
`CMAKE_PREFIX_PATH` 不含 `~/.local`，因此装到 `/usr/local` 后，未额外设置前缀的
工程会找到系统副本。

## 6. 验证

```bash
slint-compiler --version
```

期望输出：

```text
slint-compiler 1.19.0
```

确认 CMake 能找到包（系统安装把前缀改成 `/usr/local`，用户安装使用 `$HOME/.local`）：

```bash
cmake --find-package \
  -DNAME=Slint \
  -DCOMPILER_ID=GNU \
  -DLANGUAGE=CXX \
  -DMODE=EXIST \
  -DCMAKE_PREFIX_PATH="$HOME/.local"
```

找到时输出 `Slint found.`。检查 `/usr/local` 那一份时去掉 `-DCMAKE_PREFIX_PATH`。

## 7. 在 CMake 工程中使用

```cmake
find_package(Slint REQUIRED)
target_link_libraries(my_application PRIVATE Slint::Slint)
slint_target_sources(my_application my_application_ui.slint)
```

使用 `~/.local` 中的副本时，配置工程要带上：

```bash
cmake -DCMAKE_PREFIX_PATH="$HOME/.local" ...
```
