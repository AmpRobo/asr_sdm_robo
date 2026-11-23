#!/usr/bin/env bash
# Set up development environment for asr_sdm_robo.
# Usage: ./setup-dev-env.sh [OPTIONS]
# Note: -y option is only for CI.

set -euo pipefail

option_common=false
option_ros=false
option_nvidia=false
option_no_nvidia=false
option_no_cuda_drivers=false
option_runtime=false
option_yes=false
option_list=false
option_rebuild=false
option_purge_nvidia=false

# Set when apt sources change so the next apt_update really refreshes the index
apt_index_stale=true

# The only ROS 2 distribution supported on Ubuntu 24.04 by this workspace
ros_distro="jazzy"
# Keeps DDS traffic of this workspace separate from other ROS 2 machines on the LAN
ros_domain_id="100"
# Used when the ros-apt-source release cannot be queried from GitHub
ros_apt_source_fallback_version="1.2.0"

# Versions of the NVIDIA stack (see src/asr_sdm_universe/common/tensorrt_common/README.md)
# The driver is the newest nvidia-driver-*-open in Ubuntu's restricted archive
# (prebuilt kernel modules, no DKMS). CUDA must not be newer than the CUDA version
# that driver supports ("CUDA UMD Version" in nvidia-smi), otherwise PTX JIT fails
# with "the provided PTX was compiled with an unsupported toolchain". cuDNN and
# TensorRT are the newest NVIDIA CUDA-repo builds for that CUDA version.
# OpenCV 4.14.0 is the newest 4.x release; 5.x changes the API.
nvidia_driver_branch="615"      # newest nvidia-driver-*-open in Ubuntu noble; supports CUDA <= 13.4
cuda_version="13.4"
cuda_apt_suffix="13-4"          # apt package suffix for CUDA 13.4
cudnn_cuda_major="13"           # cuDNN 9 flavour matching the CUDA major version
tensorrt_version="11.4.0"       # newest TensorRT with a +cuda13.4 build
tensorrt_cuda_tag="cuda${cuda_version}"   # TensorRT build to pick, e.g. 11.4.0.106-1+cuda13.4
opencv_version="4.14.0"

# Everything this script downloads or builds lives in a dependency/ folder next
# to the repository (../dependency, beside asr_sdm_robo/): source archives and
# trees, build directories, apt repo .debs and keys, the rustup installer and
# cargo's crate cache. Set ASR_SDM_DEPENDENCY_DIR to use another location.
# COLCON_IGNORE in it keeps colcon from treating the trees as packages if the
# parent folder is ever built as a workspace.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dependency_dir="${ASR_SDM_DEPENDENCY_DIR:-$(dirname "${script_dir}")/dependency}"
# Where older versions of this script kept it (moved on first run)
legacy_dependency_dir="${script_dir}/dependency"
periphery_repo_url="https://github.com/AmpRobo/c-periphery.git"
periphery_src_dir="${dependency_dir}/c-periphery"
periphery_version="2.5.0"
periphery_install_prefix="/usr/local"
opencv_src_dir="${dependency_dir}/opencv-${opencv_version}"
opencv_contrib_src_dir="${dependency_dir}/opencv_contrib-${opencv_version}"
# cargo registry/git cache used when building Slint (the Rust toolchain stays in ~/.rustup)
cargo_cache_dir="${dependency_dir}/cargo"
# Slint C++ API, built from source (its runtime is Rust, so it needs cargo)
slint_version="1.19.0"
slint_src_dir="${dependency_dir}/slint-${slint_version}"
slint_install_prefix="/usr/local"
# Minimum Rust installed by --common (what Slint needs)
rust_min_version="1.95"
# matplot++ (C++ plotting), built from source
matplotpp_version="1.2.2"
matplotpp_repo_url="https://github.com/alandefreitas/matplotplusplus.git"
matplotpp_src_dir="${dependency_dir}/matplotplusplus-${matplotpp_version}"
matplotpp_install_prefix="/usr/local"
# Pinocchio stack built from source, in dependency order eigenpy -> Coal -> Pinocchio.
# Pinned to a combination that satisfies each project's pixi.toml constraints
# (Pinocchio 4.1.0 needs eigenpy >= 3.7.0 and Coal >= 3); bump them together.
eigenpy_version="3.13.0"
coal_version="3.0.4"
pinocchio_version="4.1.0"
eigenpy_repo_url="https://github.com/stack-of-tasks/eigenpy.git"
coal_repo_url="https://github.com/coal-library/coal.git"
pinocchio_repo_url="https://github.com/stack-of-tasks/pinocchio.git"
pinocchio_install_prefix="${PINOCCHIO_INSTALL_PREFIX:-/opt/openrobots}"
# PINOCCHIO_BUILD_COLLISION=ON builds Pinocchio's Coal-based collision support
pinocchio_build_collision="${PINOCCHIO_BUILD_COLLISION:-OFF}"

# CUDA library search paths, filled in by install_opencv_cuda
cuda_lib_dirs=()

# Env exports written to ~/.bashrc at the end of the NVIDIA role
nvidia_env_content=""

print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message"
    echo "  -h                      Display this help message"
    echo "  --list                  List all installable components"
    echo "  --common                Install common development packages (toolchain, git, python3, CLI tools,"
    echo "                          Coral GUI, Rust >= ${rust_min_version}, Slint ${slint_version} C++, matplot++ ${matplotpp_version},"
    echo "                          Pinocchio ${pinocchio_version} / eigenpy ${eigenpy_version} / Coal ${coal_version},"
    echo "                          c-periphery ${periphery_version} static library, all from source)"
    echo "  --ros                   Install ROS 2 ${ros_distro} desktop + rosdep/colcon (Ubuntu 24.04)"
    echo "  --nvidia                Install the NVIDIA stack (driver ${nvidia_driver_branch}, CUDA ${cuda_version},"
    echo "                          cuBLAS, NPP, cuDNN, TensorRT ${tensorrt_version}, OpenCV ${opencv_version} + CUDA)"
    echo "  -y                      Use non-interactive mode"
    echo "  -v                      Enable debug outputs"
    echo "  --purge-nvidia          Purge every NVIDIA-related package (driver, kernel modules, CUDA, cuDNN,"
    echo "                          TensorRT, NVIDIA apt repos), the TensorRT pin, OpenCV + CUDA and the"
    echo "                          ~/.bashrc exports. Combined with --nvidia, the stack is reinstalled afterwards"
    echo "  --no-nvidia             Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers       Keep the current NVIDIA driver (skip the driver install)"
    echo "  --runtime               Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --rebuild               Rebuild source-built libraries (Slint, matplot++, Pinocchio stack,"
    echo "                          c-periphery, OpenCV)"
    echo "                          even when the same version is already installed"
    echo "  --data-dir              Set data directory (default: $HOME/asr_sdm_data)"
    echo "  --download-artifacts    Download artifacts"
    echo "  --module                Specify the module (default: all)"
    echo "  --ros-distro            ROS distribution to install (only '${ros_distro}' is supported)"
    echo ""
}

print_list() {
    echo "Installable components:"
    echo ""
    echo "  Flag          Component    Description"
    echo "  ----          ---------    -----------"
    echo "  --common      Common       Install build toolchain, git, python3, CLI tools, Coral GUI libraries,"
    echo "                             Rust stable >= ${rust_min_version} via rustup (into ~/.cargo, ~/.rustup)"
    echo "                             and the source-built libraries below"
    echo "  --ros         ROS 2        Install ROS 2 ${ros_distro} desktop + rosdep/colcon (Ubuntu 24.04),"
    echo "                             and set ROS_DOMAIN_ID=${ros_domain_id} in ~/.bashrc"
    echo "  --nvidia      NVIDIA       Install the GPU stack below (Ubuntu 24.04, x86_64)"
    echo "  --purge-nvidia             Purge every NVIDIA-related package and OpenCV + CUDA (runs before --nvidia)"
    echo ""
    echo "  Source-built components of --common (sources and builds in ${dependency_dir}):"
    echo "    Slint           ${slint_version} C++ API into ${slint_install_prefix}"
    echo "    matplot++       ${matplotpp_version} shared library into ${matplotpp_install_prefix}"
    echo "    eigenpy         ${eigenpy_version} into ${pinocchio_install_prefix}"
    echo "    Coal            ${coal_version} into ${pinocchio_install_prefix} (with Python bindings)"
    echo "    Pinocchio       ${pinocchio_version} into ${pinocchio_install_prefix} (with Python bindings;"
    echo "                    collision support: PINOCCHIO_BUILD_COLLISION=${pinocchio_build_collision})"
    echo "    c-periphery     ${periphery_version} static library into ${periphery_install_prefix}"
    echo ""
    echo "  Components of --nvidia:"
    echo "    NVIDIA driver   ${nvidia_driver_branch} (nvidia-driver-${nvidia_driver_branch}-open + prebuilt kernel modules; skip with --no-cuda-drivers)"
    echo "    CUDA Toolkit    ${cuda_version} (cuda-toolkit-${cuda_apt_suffix})"
    echo "    CUDA cuBLAS     libcublas-${cuda_apt_suffix}"
    echo "    CUDA NPP        libnpp-${cuda_apt_suffix} (plus cuFFT, required by OpenCV CUDA)"
    echo "    cuDNN           libcudnn9-cuda-${cudnn_cuda_major}"
    echo "    TensorRT        ${tensorrt_version} (needs the NVIDIA TensorRT local apt repo)"
    echo "    OpenCV + CUDA   ${opencv_version} into ~/.local/opencv-cuda"
    echo "                    (source in ${dependency_dir}/opencv-${opencv_version} and"
    echo "                     ${dependency_dir}/opencv_contrib-${opencv_version})"
    echo ""
    echo "Examples:"
    echo "  ./setup-dev-env.sh --list"
    echo "  ./setup-dev-env.sh --common"
    echo "  ./setup-dev-env.sh --ros"
    echo "  ./setup-dev-env.sh --nvidia"
    echo "  ./setup-dev-env.sh --nvidia --runtime --no-cuda-drivers"
    echo "  ./setup-dev-env.sh --nvidia --rebuild"
    echo "  ./setup-dev-env.sh --purge-nvidia"
    echo "  ./setup-dev-env.sh --purge-nvidia --nvidia"
    echo "  ./setup-dev-env.sh --ros --common --nvidia -y"
}

run_cmd() {
    echo "Running: $*"
    "$@"
}

confirm_or_exit() {
    if [[ "${option_yes}" == true ]]; then
        export DEBIAN_FRONTEND=noninteractive
        return
    fi

    if [[ "${option_purge_nvidia}" == true ]]; then
        echo -e "\e[31mThis will PURGE all NVIDIA packages (driver, CUDA, cuDNN, TensorRT) and OpenCV + CUDA.\e[m"
    fi
    if [[ "${option_common}" == true || "${option_ros}" == true || "${option_nvidia}" == true ]]; then
        echo -e "\e[33mThis will install the selected development packages.\e[m"
    fi
    read -rp "> Continue? [y/N] " answer
    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi
}

apt_install() {
    if [[ "${option_yes}" == true ]]; then
        sudo apt-get --yes "$@"
    else
        sudo apt-get "$@"
    fi
}

# apt-get update once per run, plus again after a step adds or changes an apt source
apt_update() {
    if [[ "${apt_index_stale}" == true ]]; then
        apt_install update
        apt_index_stale=false
    fi
}

# Call after adding or changing an apt source
mark_apt_sources_changed() {
    apt_index_stale=true
}

ensure_sudo() {
    # Minimal containers run as root without sudo; every step below calls sudo
    if ! command -v sudo >/dev/null 2>&1; then
        apt-get -y update
        apt-get -y install sudo
    fi
}

# Prints "-G Ninja" (one word per line) when ninja is available, so callers can
# use: mapfile -t generator < <(cmake_generator_args)
cmake_generator_args() {
    if command -v ninja >/dev/null 2>&1; then
        printf '%s\n' -G Ninja
    fi
}

install_common() {
    local -a common_packages

    if ! command -v apt-get >/dev/null 2>&1; then
        echo -e "\e[33mSkipping common packages: apt-get is not available.\e[m"
        return 0
    fi

    common_packages=(
        # Toolchain and build systems
        build-essential
        cmake
        ninja-build
        pkg-config
        ccache
        gdb
        clang-format
        # Version control
        git
        git-lfs
        # Fetching and archives
        ca-certificates
        curl
        wget
        gnupg
        unzip
        zip
        # Python
        python3
        python3-pip
        python3-venv
        python3-dev
        # Qt 6 and the QML modules needed at runtime by Qt Quick UIs
        qt6-base-dev
        qt6-declarative-dev
        qt6-tools-dev
        qml6-module-qtqml
        qml6-module-qtqml-workerscript
        qml6-module-qtquick
        qml6-module-qtquick-controls
        qml6-module-qtquick-layouts
        qml6-module-qtquick-templates
        qml6-module-qtquick-window
        # Coral GUI (OpenSceneGraph / osgOcean and VulkanSceneGraph)
        libopenscenegraph-dev
        libfftw3-dev
        libvulkan-dev
        libassimp-dev
        glslang-dev
        glslang-tools
        libxcb1-dev
        # Point Cloud Library and optional io pcap support
        libpcl-dev
        libpcap-dev
        # Logging
        libgoogle-glog-dev
        # Pinocchio / eigenpy / Coal build dependencies
        python3-numpy
        python3-pybind11
        libeigen3-dev
        libboost-all-dev
        liburdfdom-dev
        libtinyxml2-dev
        # c-periphery (uses the kernel's linux/*.h userspace headers)
        linux-libc-dev
        # matplot++ build and runtime dependencies (gnuplot is its plotting backend)
        gnuplot
        libpng-dev
        libjpeg-dev
        libtiff-dev
        libblas-dev
        liblapack-dev
        zlib1g-dev
        # Slint C++ build dependencies
        libfontconfig-dev
        libxkbcommon-dev
        libxcb-shape0-dev
        libxcb-xfixes0-dev
        libssl-dev
        # Everyday CLI tools
        vim
        tmux
        htop
        tree
        jq
        ripgrep
        # Distro and hardware inspection
        lsb-release
        software-properties-common
        net-tools
        iputils-ping
        pciutils
        usbutils
    )

    echo -e "\e[36mInstalling common development packages...\e[m"

    apt_update

    # xacro is only packaged in the ROS apt repository (set up by --ros)
    if apt-cache show "ros-${ros_distro}-xacro" >/dev/null 2>&1; then
        common_packages+=("ros-${ros_distro}-xacro")
    else
        echo -e "\e[33mSkipping xacro: 'ros-${ros_distro}-xacro' is not available. Run with --ros first to add the ROS apt repository.\e[m"
    fi

    apt_install install "${common_packages[@]}"

    ensure_rust_toolchain
    install_slint
    install_periphery
    install_matplotpp
    install_pinocchio_stack

    echo -e "\e[32mDone.\e[0m Common packages installed."
}

# Home directory of the user behind sudo (or the current user)
invoking_user_home() {
    if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        getent passwd "${SUDO_USER}" | cut -d: -f6
    else
        printf '%s' "${HOME}"
    fi
}

# Prints the version of <rustc>, or nothing when it cannot run
rustc_version_of() {
    "$1" --version 2>/dev/null | awk '{print $2}' || true
}

version_at_least() {
    # version_at_least <have> <need>
    [[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -n1)" == "$2" ]]
}

# Part of --common: make sure the invoking user has a rustup-managed Rust
# >= rust_min_version (the minimum Slint needs). rustup's installer adds
# `. "$HOME/.cargo/env"` to ~/.bashrc so new shells find cargo.
# The rustup installer script is saved into dependency/; the toolchain itself
# lives in ~/.cargo and ~/.rustup as usual.
ensure_rust_toolchain() {
    local user_home cargo_bin rustup_init have
    user_home="$(invoking_user_home)"
    cargo_bin="${user_home}/.cargo/bin"

    if [[ ! -x "${cargo_bin}/rustup" ]]; then
        download_to_dependency https://sh.rustup.rs rustup-init.sh
        rustup_init="${dependency_dir}/rustup-init.sh"
        echo -e "\e[36mInstalling Rust (stable) with rustup...\e[m"
        run_cmd run_as_invoking_user sh "${rustup_init}" -y --default-toolchain stable --profile minimal
    fi

    have="$(rustc_version_of "${cargo_bin}/rustc")"
    if [[ -z "${have}" ]] || ! version_at_least "${have}" "${rust_min_version}"; then
        echo -e "\e[36mRust ${have:-unknown} is older than ${rust_min_version}; updating the stable toolchain and making it the default...\e[m"
        run_cmd run_as_invoking_user "${cargo_bin}/rustup" update stable
        run_cmd run_as_invoking_user "${cargo_bin}/rustup" default stable
        have="$(rustc_version_of "${cargo_bin}/rustc")"
    fi

    if [[ -z "${have}" ]] || ! version_at_least "${have}" "${rust_min_version}"; then
        echo -e "\e[31mRust >= ${rust_min_version} is required for Slint, found ${have:-none}.\e[m" >&2
        return 1
    fi
    echo "Using Rust ${have} from ${cargo_bin}."
}

install_slint() {
    local build_dir="${slint_src_dir}/build"
    local compiler="${slint_install_prefix}/bin/slint-compiler"
    local user_home cargo_bin

    if [[ "${option_rebuild}" != true && -x "${compiler}" && -d "${slint_install_prefix}/lib/cmake/Slint" ]] \
        && "${compiler}" --version 2>/dev/null | grep -qw "${slint_version}"; then
        echo "Slint ${slint_version} already installed in ${slint_install_prefix}, skipping."
        return 0
    fi

    echo -e "\e[36mBuilding Slint ${slint_version} C++ API...\e[m"

    # Rust comes from ensure_rust_toolchain, run earlier in install_common
    user_home="$(invoking_user_home)"
    cargo_bin="${user_home}/.cargo/bin"

    # GitHub extracts tag v<version> as slint-<version>/
    ensure_github_tag_source "slint-ui/slint" "v${slint_version}" "slint-${slint_version}"

    echo "  SOURCE_DIR      = ${slint_src_dir}"
    echo "  BUILD_DIR       = ${build_dir}"
    echo "  INSTALL_PREFIX  = ${slint_install_prefix}"

    # Configure and build as the invoking user so cargo uses their toolchain;
    # CARGO_HOME keeps the downloaded crates in dependency/. Only install needs root.
    mkdir -p "${cargo_cache_dir}"
    reset_stale_cmake_cache "${slint_src_dir}" "${build_dir}"
    run_cmd run_as_invoking_user env PATH="${cargo_bin}:${PATH}" CARGO_HOME="${cargo_cache_dir}" \
        cmake -S "${slint_src_dir}" -B "${build_dir}" -G Ninja \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX="${slint_install_prefix}"
    run_cmd run_as_invoking_user env PATH="${cargo_bin}:${PATH}" CARGO_HOME="${cargo_cache_dir}" \
        cmake --build "${build_dir}" --parallel "$(nproc)"
    run_cmd sudo cmake --install "${build_dir}"
    fix_dependency_owner

    echo -e "\e[32mDone.\e[0m Slint installed. Verify: ${compiler} --version"
}

# ensure_git_tag_source <repo_url> <tag> <dir_name> [submodule_path]: shallow clone of
# <tag> with its submodules (or only [submodule_path]) into dependency/<dir_name>.
# Used instead of GitHub zips for projects whose build needs submodules
# (jrl-cmakemodules in cmake/ for the Pinocchio stack).
ensure_git_tag_source() {
    local repo_url="$1"
    local tag="$2"
    local dest_dir="${dependency_dir}/$3"
    local recurse="--recurse-submodules${4:+=$4}"

    ensure_dependency_dir

    if [[ -f "${dest_dir}/CMakeLists.txt" ]]; then
        echo "$3 source already present in ${dest_dir}, skipping clone."
        return 0
    fi

    if [[ -e "${dest_dir}" ]]; then
        echo -e "\e[31m${dest_dir} exists but has no CMakeLists.txt; not overwriting it.\e[m" >&2
        return 1
    fi

    echo -e "\e[36mCloning ${repo_url} ${tag} into ${dest_dir}...\e[m"
    run_cmd run_as_invoking_user git clone --depth 1 --branch "${tag}" \
        "${recurse}" --shallow-submodules "${repo_url}" "${dest_dir}"
    fix_dependency_owner
}

# source_build_installed <src_dir> <prefix> <package_dir>: true when cmake_build_install
# already installed this (versioned) source tree into <prefix> and the CMake package
# is still there, unless --rebuild is set. A stamp is used instead of the package's
# reported version because some tags (matplot++ v1.2.2) declare an older one.
source_build_installed() {
    local stamp="$1/build/.installed-by-setup-dev-env"
    local prefix="$2"
    local package_dir="$3"

    [[ "${option_rebuild}" != true && -f "${stamp}" ]] || return 1
    grep -qxF "${prefix}" "${stamp}" || return 1
    compgen -G "${prefix}/lib/cmake/${package_dir}" >/dev/null \
        || compgen -G "${prefix}/lib/*/cmake/${package_dir}" >/dev/null
}

# cmake_build_install <src_dir> <prefix> <jobs> [extra cmake args...]:
# configure and build <src_dir>/build as the invoking user, then install into
# <prefix> (with sudo when <prefix> is not writable).
# reset_stale_cmake_cache <src_dir> <build_dir>: drop a CMake cache that was
# configured for another source path (e.g. after dependency/ was moved), which
# cmake would otherwise refuse to reuse. Sources and stamp files are kept.
reset_stale_cmake_cache() {
    local src_dir="$1"
    local build_dir="$2"
    local cached

    [[ -f "${build_dir}/CMakeCache.txt" ]] || return 0
    cached="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "${build_dir}/CMakeCache.txt")"
    if [[ -n "${cached}" && "${cached}" != "${src_dir}" ]]; then
        echo "CMake cache in ${build_dir} was made for ${cached}; reconfiguring."
        run_cmd rm -rf "${build_dir}/CMakeCache.txt" "${build_dir}/CMakeFiles"
    fi
}

cmake_build_install() {
    local src_dir="$1"
    local prefix="$2"
    local jobs="$3"
    local build_dir="${src_dir}/build"
    local probe="${prefix}"
    local -a generator
    shift 3

    mapfile -t generator < <(cmake_generator_args)
    reset_stale_cmake_cache "${src_dir}" "${build_dir}"
    run_cmd run_as_invoking_user env CMAKE_PREFIX_PATH="${prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}" \
        cmake -S "${src_dir}" -B "${build_dir}" "${generator[@]}" \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX="${prefix}" \
        "$@"
    run_cmd run_as_invoking_user cmake --build "${build_dir}" --parallel "${jobs}"

    # Walk up to the first existing directory to decide whether sudo is needed
    while [[ ! -e "${probe}" && "${probe}" != "/" ]]; do
        probe="$(dirname "${probe}")"
    done
    if [[ -w "${probe}" ]]; then
        run_cmd cmake --install "${build_dir}"
    else
        run_cmd sudo cmake --install "${build_dir}"
        run_cmd sudo ldconfig
    fi
    echo "${prefix}" > "${build_dir}/.installed-by-setup-dev-env"
    fix_dependency_owner
}

install_matplotpp() {
    if source_build_installed "${matplotpp_src_dir}" "${matplotpp_install_prefix}" "Matplot++"; then
        echo "matplot++ ${matplotpp_version} already installed in ${matplotpp_install_prefix}, skipping."
        return 0
    fi

    echo -e "\e[36mBuilding matplot++ ${matplotpp_version}...\e[m"
    ensure_git_tag_source "${matplotpp_repo_url}" "v${matplotpp_version}" "matplotplusplus-${matplotpp_version}"
    cmake_build_install "${matplotpp_src_dir}" "${matplotpp_install_prefix}" "$(nproc)" \
        -D CMAKE_CXX_STANDARD=17 \
        -D MATPLOTPP_BUILD_EXAMPLES=OFF \
        -D MATPLOTPP_BUILD_TESTS=OFF \
        -D MATPLOTPP_BUILD_EXPERIMENTAL_OPENGL_BACKEND=OFF \
        -D BUILD_SHARED_LIBS=ON

    echo -e "\e[32mDone.\e[0m matplot++ ${matplotpp_version} installed. Use in CMake: find_package(Matplot++ REQUIRED)"
}

# register_openrobots_prefix <prefix> <python_version>: make the Pinocchio stack in
# <prefix> visible without the ~/.bashrc exports, which shells opened before the
# install, non-interactive shells (~/.bashrc returns early), IDEs and CI never load:
#   - CMake: entries in the user package registry (~/.cmake/packages/<name>/),
#     which find_package() consults by default, so colcon finds pinocchio/coal/eigenpy
#   - runtime linker: /etc/ld.so.conf.d entry + ldconfig
#   - Python: a .pth file in the system site-packages directory
register_openrobots_prefix() {
    local prefix="$1"
    local python_version="$2"
    local user_home registry_dir pkg config_dir site_dir
    local ld_conf="/etc/ld.so.conf.d/asr-sdm-openrobots.conf"
    local pth_name="asr-sdm-openrobots.pth"

    user_home="$(invoking_user_home)"
    for pkg in eigenpy coal pinocchio; do
        config_dir="${prefix}/lib/cmake/${pkg}"
        [[ -d "${config_dir}" ]] || continue
        registry_dir="${user_home}/.cmake/packages/${pkg}"
        run_as_invoking_user mkdir -p "${registry_dir}"
        printf '%s\n' "${config_dir}" | run_as_invoking_user tee "${registry_dir}/asr_sdm_robo" >/dev/null
    done
    echo "Registered eigenpy / coal / pinocchio in the CMake user package registry (${user_home}/.cmake/packages)."

    if [[ "$(cat "${ld_conf}" 2>/dev/null || true)" != "${prefix}/lib" ]]; then
        echo "Running: sudo tee ${ld_conf}"
        echo "${prefix}/lib" | sudo tee "${ld_conf}" >/dev/null
    fi
    run_cmd sudo ldconfig

    # /usr/local/lib/python3.X/dist-packages is on sys.path of the system python3
    site_dir="/usr/local/lib/python${python_version}/dist-packages"
    if [[ -d "${prefix}/lib/python${python_version}/site-packages" ]]; then
        run_cmd sudo mkdir -p "${site_dir}"
        echo "Running: sudo tee ${site_dir}/${pth_name}"
        echo "${prefix}/lib/python${python_version}/site-packages" | sudo tee "${site_dir}/${pth_name}" >/dev/null
    fi
}

install_pinocchio_stack() {
    local python_bin
    local python_version
    local prefix="${pinocchio_install_prefix}"
    # Pinocchio's Python bindings need several GB of RAM per compile job
    local jobs=$(( $(nproc) / 2 ))
    ((jobs >= 1)) || jobs=1
    local -a common_args

    python_bin="$(command -v python3)"
    python_version="$("${python_bin}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    common_args=(
        -D BUILD_TESTING=OFF
        -D INSTALL_DOCUMENTATION=OFF
        -D PYTHON_EXECUTABLE="${python_bin}"
    )

    if dpkg -s robotpkg-pinocchio >/dev/null 2>&1; then
        echo -e "\e[33mWarning: robotpkg-pinocchio (apt) is installed and shares ${prefix} with this source build.\e[m"
        echo "Remove it first to avoid mixed files: sudo apt remove 'robotpkg-*pinocchio' 'robotpkg-*coal' 'robotpkg-*eigenpy'"
    fi

    echo -e "\e[36mSetting up eigenpy ${eigenpy_version}, Coal ${coal_version} and Pinocchio ${pinocchio_version} in ${prefix} (${jobs} build jobs)...\e[m"

    if source_build_installed "${dependency_dir}/eigenpy-${eigenpy_version}" "${prefix}" eigenpy; then
        echo "eigenpy ${eigenpy_version} already installed, skipping."
    else
        ensure_git_tag_source "${eigenpy_repo_url}" "v${eigenpy_version}" "eigenpy-${eigenpy_version}"
        cmake_build_install "${dependency_dir}/eigenpy-${eigenpy_version}" "${prefix}" "${jobs}" \
            "${common_args[@]}" \
            -D BUILD_TESTING_SCIPY=OFF \
            -D GENERATE_PYTHON_STUBS=OFF
    fi

    if source_build_installed "${dependency_dir}/coal-${coal_version}" "${prefix}" coal; then
        echo "Coal ${coal_version} already installed, skipping."
    else
        ensure_git_tag_source "${coal_repo_url}" "v${coal_version}" "coal-${coal_version}"
        cmake_build_install "${dependency_dir}/coal-${coal_version}" "${prefix}" "${jobs}" \
            "${common_args[@]}" \
            -D BUILD_PYTHON_INTERFACE=ON \
            -D BUILD_STANDALONE_PYTHON_INTERFACE=OFF \
            -D GENERATE_PYTHON_STUBS=OFF \
            -D COAL_HAS_QHULL=OFF \
            -D COAL_ENABLE_LOGGING=OFF
    fi

    if source_build_installed "${dependency_dir}/pinocchio-${pinocchio_version}" "${prefix}" pinocchio; then
        echo "Pinocchio ${pinocchio_version} already installed, skipping."
    else
        # Only the cmake/ submodule; models/example-robot-data is for tests and examples
        ensure_git_tag_source "${pinocchio_repo_url}" "v${pinocchio_version}" "pinocchio-${pinocchio_version}" cmake
        cmake_build_install "${dependency_dir}/pinocchio-${pinocchio_version}" "${prefix}" "${jobs}" \
            "${common_args[@]}" \
            -D BUILD_EXAMPLES=OFF \
            -D BUILD_PYTHON_INTERFACE=ON \
            -D BUILD_WITH_COLLISION_SUPPORT="${pinocchio_build_collision}"
    fi

    register_openrobots_prefix "${prefix}" "${python_version}"

    # Replaces the block written by the robotpkg install of earlier versions
    remove_bashrc_block "pinocchio robotpkg setup" || true
    write_bashrc_block "pinocchio source install setup" "export PATH=${prefix}/bin:\$PATH
export PKG_CONFIG_PATH=${prefix}/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=${prefix}/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=${prefix}/lib/python${python_version}/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=${prefix}:\$CMAKE_PREFIX_PATH"

    echo -e "\e[32mDone.\e[0m Pinocchio stack installed into ${prefix}."
    echo "Reload: source ~/.bashrc  |  Verify: python3 -c 'import pinocchio; print(pinocchio.__version__)'"
}

# The user who should own dependency/: the caller behind sudo, or the current
# user when not root. Empty when running as plain root (nothing to hand back).
dependency_owner() {
    local user=""
    if [[ "${EUID}" -eq 0 ]]; then
        if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
            user="${SUDO_USER}"
        fi
    else
        user="$(id -un)"
    fi
    [[ -n "${user}" ]] && echo "${user}:$(id -gn "${user}")"
    return 0
}

# Hand dependency/ (sources, archives, build dirs) back to the invoking user so
# it is never left root-owned after a sudo run or a `sudo cmake --install`.
fix_dependency_owner() {
    local owner
    [[ -d "${dependency_dir}" ]] || return 0
    owner="$(dependency_owner)"
    [[ -n "${owner}" ]] || return 0

    # Skip the recursive chown when everything already belongs to the user
    if [[ -z "$(find "${dependency_dir}" ! -user "${owner%%:*}" -print -quit 2>/dev/null)" ]]; then
        return 0
    fi

    echo "Setting owner of ${dependency_dir} to ${owner}..."
    if [[ "${EUID}" -eq 0 ]]; then
        chown -R "${owner}" "${dependency_dir}"
    else
        sudo chown -R "${owner}" "${dependency_dir}"
    fi
}

# Older versions kept dependency/ inside the repository. Move it next to the
# repository once so finished builds (and their skip stamps) are kept.
migrate_legacy_dependency_dir() {
    [[ -d "${legacy_dependency_dir}" ]] || return 0
    [[ "${legacy_dependency_dir}" != "${dependency_dir}" ]] || return 0
    if [[ -e "${dependency_dir}" ]]; then
        echo -e "\e[33mWarning: both ${legacy_dependency_dir} and ${dependency_dir} exist; using ${dependency_dir}. Remove the old one when no longer needed.\e[m"
        return 0
    fi
    echo "Moving ${legacy_dependency_dir} to ${dependency_dir}..."
    run_cmd mkdir -p "$(dirname "${dependency_dir}")"
    run_cmd mv "${legacy_dependency_dir}" "${dependency_dir}"
}

ensure_dependency_dir() {
    # Keep a COLCON_IGNORE marker so `colcon build` from the workspace root does not
    # treat third-party trees under dependency/ as packages.
    mkdir -p "${dependency_dir}"
    fix_dependency_owner
    if [[ ! -f "${dependency_dir}/COLCON_IGNORE" ]]; then
        touch "${dependency_dir}/COLCON_IGNORE"
    fi
}

# download_to_dependency <url> <file_name>: fetch <url> into dependency/<file_name>
# unless it is already there. A .part file keeps an interrupted download from
# being mistaken for a complete one on the next run.
download_to_dependency() {
    local url="$1"
    local dest="${dependency_dir}/$2"

    ensure_dependency_dir
    if [[ -s "${dest}" ]]; then
        echo "Using existing ${dest}."
        return 0
    fi

    echo -e "\e[36mDownloading ${url} into ${dependency_dir}...\e[m"
    if ! run_cmd curl -fL --retry 3 --retry-delay 2 --proto-redir '=https' -o "${dest}.part" "${url}"; then
        rm -f "${dest}.part"
        echo -e "\e[31mFailed to download ${url}\e[m" >&2
        return 1
    fi
    mv "${dest}.part" "${dest}"
    fix_dependency_owner
}

# Download a GitHub tag zip into dependency/<dir_name> when that tree is missing.
# GitHub extracts the archive as <repo>-<tag>/, which must match dir_name.
ensure_github_tag_source() {
    local repo="$1"
    local tag="$2"
    local dir_name="$3"
    # File whose presence marks a complete source tree (opencv_contrib has no top-level CMakeLists.txt)
    local marker="${4:-CMakeLists.txt}"
    local dest_dir="${dependency_dir}/${dir_name}"
    local zip_path="${dependency_dir}/${dir_name}.zip"
    local url="https://github.com/${repo}/archive/refs/tags/${tag}.zip"

    ensure_dependency_dir

    if [[ -f "${dest_dir}/${marker}" ]]; then
        echo "${dir_name} source already present in ${dest_dir}, skipping download."
        return 0
    fi

    if [[ -e "${dest_dir}" ]]; then
        echo -e "\e[31m${dest_dir} exists but has no ${marker}; not overwriting it.\e[m" >&2
        return 1
    fi

    download_to_dependency "${url}" "${dir_name}.zip"

    echo "Extracting ${zip_path}..."
    run_cmd unzip -q -d "${dependency_dir}" "${zip_path}"

    if [[ ! -f "${dest_dir}/${marker}" ]]; then
        echo -e "\e[31mFailed to extract ${repo} ${tag} into ${dest_dir}.\e[m" >&2
        return 1
    fi

    fix_dependency_owner
}

ensure_periphery_source() {
    ensure_dependency_dir

    if [[ -f "${periphery_src_dir}/CMakeLists.txt" ]]; then
        echo "c-periphery source already present in ${periphery_src_dir}, skipping clone."
        return 0
    fi

    if [[ -e "${periphery_src_dir}" ]]; then
        echo -e "\e[31m${periphery_src_dir} exists but has no CMakeLists.txt; not overwriting it.\e[m" >&2
        return 1
    fi

    echo -e "\e[36mCloning c-periphery into ${periphery_src_dir}...\e[m"
    run_cmd run_as_invoking_user git clone --depth 1 "${periphery_repo_url}" "${periphery_src_dir}"
    fix_dependency_owner
}

ensure_opencv_source() {
    ensure_github_tag_source "opencv/opencv" "${opencv_version}" "opencv-${opencv_version}"
    ensure_github_tag_source "opencv/opencv_contrib" "${opencv_version}" "opencv_contrib-${opencv_version}" "modules/README.md"
}

install_periphery() {
    local static_lib=""
    local candidate
    local multiarch=""

    if source_build_installed "${periphery_src_dir}" "${periphery_install_prefix}" periphery; then
        echo "c-periphery already installed in ${periphery_install_prefix}, skipping."
        return 0
    fi

    echo -e "\e[36mInstalling c-periphery ${periphery_version} as a static library...\e[m"
    ensure_periphery_source

    # CMAKE_INSTALL_LIBDIR=lib keeps libperiphery.a in /usr/local/lib so the
    # linker and find_package(periphery) both find it without a multiarch path.
    cmake_build_install "${periphery_src_dir}" "${periphery_install_prefix}" "$(nproc)" \
        -D CMAKE_INSTALL_LIBDIR=lib \
        -D BUILD_SHARED_LIBS=OFF \
        -D BUILD_TESTS=OFF

    if command -v dpkg-architecture >/dev/null 2>&1; then
        multiarch="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
    fi

    for candidate in \
        "${periphery_install_prefix}/lib/libperiphery.a" \
        "${periphery_install_prefix}/lib64/libperiphery.a" \
        "${periphery_install_prefix}/lib/${multiarch}/libperiphery.a"
    do
        if [[ -f "${candidate}" ]]; then
            static_lib="${candidate}"
            break
        fi
    done

    if [[ -z "${static_lib}" ]]; then
        echo -e "\e[31mFailed to install c-periphery: libperiphery.a was not found under ${periphery_install_prefix}.\e[m" >&2
        return 1
    fi

    echo -e "\e[32mDone.\e[0m c-periphery static library installed: ${static_lib}"
    echo "CMake package: find_package(periphery)  (prefix ${periphery_install_prefix})"
}

ubuntu_codename() {
    # /etc/os-release works before lsb-release is installed, lsb_release is the fallback
    local codename=""

    if [[ -r /etc/os-release ]]; then
        codename="$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME:-}}")"
    fi

    if [[ -z "${codename}" ]] && command -v lsb_release >/dev/null 2>&1; then
        codename="$(lsb_release -sc 2>/dev/null || true)"
    fi

    printf '%s' "${codename:-unknown}"
}

run_as_invoking_user() {
    # rosdep refuses to run as root, so drop back to the user behind sudo
    if [[ "${EUID}" -eq 0 && -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
        sudo -u "${SUDO_USER}" -H "$@"
    else
        "$@"
    fi
}

enable_ubuntu_universe() {
    # Matches both the classic sources.list format and the deb822 format used since 24.04
    if grep -Rqs -E '^(deb .*[[:space:]]universe([[:space:]]|$)|Components:.*[[:space:]]universe([[:space:]]|$))' \
        /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null; then
        echo "Ubuntu 'universe' component already enabled, skipping."
        return 0
    fi

    apt_install install software-properties-common
    run_cmd sudo add-apt-repository -y universe
    mark_apt_sources_changed
}

configure_utf8_locale() {
    if [[ "$(locale charmap 2>/dev/null || true)" == "UTF-8" ]]; then
        export LANG="${LANG:-en_US.UTF-8}"
        return 0
    fi

    apt_install install locales
    run_cmd sudo locale-gen en_US en_US.UTF-8
    run_cmd sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8
}

setup_ros_apt_repo() {
    local codename="$1"
    local source_version=""
    local deb_url
    local deb_name

    if [[ "$(dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null || true)" == "install ok installed" ]]; then
        echo "ros2-apt-source already installed, skipping."
        apt_update
        return 0
    fi

    echo -e "\e[36mAdding the ROS 2 apt repository...\e[m"
    apt_install install ca-certificates curl gnupg

    source_version="$(curl -fsSL --retry 3 --retry-delay 2 \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null |
        awk -F'"' '/"tag_name"/ { print $4; exit }' || true)"

    if [[ -z "${source_version}" ]]; then
        source_version="${ros_apt_source_fallback_version}"
        echo -e "\e[33mCould not query the latest ros-apt-source release; falling back to ${source_version}.\e[m"
    fi

    deb_name="ros2-apt-source_${source_version}.${codename}_all.deb"
    deb_url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${source_version}/${deb_name}"
    download_to_dependency "${deb_url}" "${deb_name}" || return 1

    if ! run_cmd sudo dpkg -i "${dependency_dir}/${deb_name}"; then
        apt_install install -f
    fi

    mark_apt_sources_changed
    apt_update
}

setup_rosdep() {
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        run_cmd sudo rosdep init
    else
        echo "rosdep already initialized, skipping 'rosdep init'."
    fi

    if ! run_as_invoking_user rosdep update --rosdistro "${ros_distro}"; then
        echo -e "\e[33m'rosdep update' failed (network?); re-run it manually before building.\e[m"
    fi
}

remove_legacy_ros_bashrc_lines() {
    # Drops the plain ROS block written by earlier versions of this script so that
    # write_bashrc_block stays the single source of truth
    local bashrc_path="${HOME}/.bashrc"
    local tmp_file

    [[ -f "${bashrc_path}" ]] || return 0
    grep -qE '^# ROS 2 [a-z]+ environment$' "${bashrc_path}" || return 0

    tmp_file="$(mktemp)"
    awk '
        /^# ROS 2 [a-z]+ environment$/ { skipping = 1; next }
        skipping == 1 && /^(source \/opt\/ros\/|source \/usr\/share\/colcon_cd\/|export _colcon_cd_root=)/ { next }
        { skipping = 0; print }
    ' "${bashrc_path}" > "${tmp_file}"
    cat "${tmp_file}" > "${bashrc_path}"
    rm -f "${tmp_file}"
    echo "Removed the legacy ROS 2 environment lines from ~/.bashrc."
}

install_ros() {
    local codename

    codename="$(ubuntu_codename)"
    if [[ "${codename}" != "noble" ]]; then
        echo -e "\e[33mSkipping ROS 2: unsupported Ubuntu (${codename}). Supported: noble (24.04).\e[m"
        return 0
    fi

    echo -e "\e[36mInstalling ROS 2 ${ros_distro}...\e[m"

    apt_update
    apt_install install \
        ca-certificates \
        curl \
        gnupg \
        git \
        build-essential \
        cmake \
        pkg-config \
        python3 \
        python3-pip \
        python3-venv

    configure_utf8_locale
    enable_ubuntu_universe

    if ! setup_ros_apt_repo "${codename}"; then
        echo -e "\e[31mSkipping ROS 2: the apt repository could not be set up.\e[m" >&2
        return 1
    fi

    if ! apt-cache show "ros-${ros_distro}-desktop" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping ROS 2: 'ros-${ros_distro}-desktop' is not available for ${codename}.\e[m"
        return 0
    fi

    # ROS packages are built against the current Ubuntu updates, so refresh them first
    apt_install upgrade
    apt_install install \
        "ros-${ros_distro}-desktop" \
        "ros-${ros_distro}-xacro" \
        ros-dev-tools \
        python3-rosdep \
        python3-colcon-common-extensions

    setup_rosdep

    remove_legacy_ros_bashrc_lines
    write_bashrc_block "asr_sdm ros2 env" "source /opt/ros/${ros_distro}/setup.bash
export ROS_DOMAIN_ID=${ros_domain_id}
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
    . /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/${ros_distro}/
fi
if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
    . /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi"

    echo -e "\e[32mDone.\e[0m ROS 2 ${ros_distro} installed (ROS_DOMAIN_ID=${ros_domain_id})."
    echo "Reload: source ~/.bashrc  |  Verify: ros2 run demo_nodes_cpp talker"
}

portable_home_path() {
    # Keep a leading home directory as a literal ${HOME} so generated env exports stay user-agnostic
    local path="$1"

    if [[ "${path}" == "${HOME}" ]]; then
        printf '${HOME}'
    elif [[ "${path}" == "${HOME}/"* ]]; then
        printf '${HOME}/%s' "${path#"${HOME}/"}"
    else
        printf '%s' "${path}"
    fi
}

portable_home_path_list() {
    # portable_home_path applied to every entry of a colon-separated path list
    local list="$1"
    local result=""
    local item

    local IFS=':'
    for item in ${list}; do
        [[ -n "${item}" ]] || continue
        if [[ -z "${result}" ]]; then
            result="$(portable_home_path "${item}")"
        else
            result="${result}:$(portable_home_path "${item}")"
        fi
    done

    printf '%s' "${result}"
}

remove_bashrc_block() {
    # remove_bashrc_block <marker>; drops "# >>> <marker> >>>" ... "# <<< <marker> <<<"
    # from ~/.bashrc. Returns 1 when the block is not there.
    local marker="$1"
    local bashrc_path="${HOME}/.bashrc"
    local begin_line="# >>> ${marker} >>>"
    local end_line="# <<< ${marker} <<<"
    local tmp_file

    [[ -f "${bashrc_path}" ]] && grep -qxF "${begin_line}" "${bashrc_path}" || return 1

    tmp_file="$(mktemp)"
    awk -v begin_line="${begin_line}" -v end_line="${end_line}" '
        $0 == begin_line { inside = 1 }
        inside == 0      { print }
        $0 == end_line   { inside = 0 }
    ' "${bashrc_path}" > "${tmp_file}"
    # Command substitution drops the trailing blank lines left behind by the removed block
    printf '%s\n' "$(cat "${tmp_file}")" > "${bashrc_path}"
    rm -f "${tmp_file}"
}

write_bashrc_block() {
    # write_bashrc_block <marker> <content>; rewrites the block when it already exists
    local marker="$1"
    local content="$2"
    local bashrc_path="${HOME}/.bashrc"

    touch "${bashrc_path}"

    if remove_bashrc_block "${marker}"; then
        echo "Refreshed '${marker}' in ~/.bashrc."
    else
        echo "Appended '${marker}' to ~/.bashrc."
    fi

    {
        echo ""
        echo "# >>> ${marker} >>>"
        echo "${content}"
        echo "# <<< ${marker} <<<"
    } >> "${bashrc_path}"
}

apt_candidate_version() {
    # apt_candidate_version <package> <version-prefix> [<substring>]
    # Prints the newest apt version starting with <version-prefix> (and containing <substring>)
    local pkg="$1"
    local prefix="$2"
    local substring="${3:-}"
    apt-cache madison "${pkg}" 2>/dev/null | awk -F'|' -v p="${prefix}" -v s="${substring}" '
        { gsub(/^[ \t]+|[ \t]+$/, "", $2); if (index($2, p) == 1 && index($2, s) > 0) { print $2; exit } }'
}

check_nvidia_prerequisites() {
    local codename
    local arch

    codename="$(ubuntu_codename)"
    arch="$(dpkg --print-architecture 2>/dev/null || echo unknown)"

    if [[ "${codename}" != "noble" ]]; then
        echo -e "\e[33mSkipping NVIDIA stack: unsupported Ubuntu (${codename}). Supported: noble (24.04).\e[m"
        return 1
    fi

    if [[ "${arch}" != "amd64" ]]; then
        echo -e "\e[33mSkipping NVIDIA stack: unsupported architecture (${arch}). Supported: amd64.\e[m"
        return 1
    fi
}

setup_cuda_apt_repo() {
    local keyring_name="cuda-keyring_1.1-1_all.deb"
    local keyring_url="https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/${keyring_name}"

    if dpkg -s cuda-keyring >/dev/null 2>&1; then
        echo "cuda-keyring already installed, skipping."
    else
        echo -e "\e[36mAdding the NVIDIA CUDA apt repository...\e[m"
        apt_update
        apt_install install ca-certificates curl gnupg
        download_to_dependency "${keyring_url}" "${keyring_name}"
        run_cmd sudo dpkg -i "${dependency_dir}/${keyring_name}"
        mark_apt_sources_changed
    fi

    apt_update
}

detect_nvidia_driver_version() {
    # Prints the installed driver version, or nothing when no driver is found
    local version=""

    if command -v nvidia-smi >/dev/null 2>&1; then
        version="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d '[:space:]' || true)"
    fi

    # A driver installed but not yet loaded (no reboot yet) is only visible below
    if [[ -z "${version}" && -r /proc/driver/nvidia/version ]]; then
        version="$(awk '{ for (i = 1; i <= NF; i++) if ($i ~ /^[0-9]+\.[0-9]+/) { print $i; exit } }' /proc/driver/nvidia/version || true)"
    fi

    if [[ -z "${version}" ]] && command -v modinfo >/dev/null 2>&1; then
        version="$(modinfo nvidia -F version 2>/dev/null | head -1 | tr -d '[:space:]' || true)"
    fi

    printf '%s' "${version}"
}

installed_nvidia_driver_package() {
    # Prints the highest installed Ubuntu driver package (e.g. nvidia-driver-615-open), or nothing
    dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n' 'nvidia-driver-*' 2>/dev/null |
        awk '$2 == "ii" && $1 ~ /^nvidia-driver-[0-9]+(-open)?$/ { print $1 }' |
        sort -t- -k3,3n | tail -1 || true   # dpkg-query fails when nothing matches
}

installed_kernel_versions() {
    # Prints every kernel that can be booted (one per line), e.g. 7.0.0-43-generic
    local image

    for image in /boot/vmlinuz-*; do
        [[ -e "${image}" ]] || continue
        printf '%s\n' "${image#/boot/vmlinuz-}"
    done
}

installed_kernel_flavours() {
    # Prints the installed kernel metapackage flavours, e.g. generic-hwe-24.04 for linux-image-generic-hwe-24.04
    dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n' 'linux-image-*' 2>/dev/null |
        awk '$2 == "ii" { sub(/^linux-image-/, "", $1); print $1 }' |
        grep -v -E '^([0-9]|unsigned-|uc-)' || true
}

kernels_missing_nvidia_module() {
    # Prints the installed kernels that have no nvidia.ko, i.e. would boot without the driver
    local kver

    while IFS= read -r kver; do
        [[ -n "${kver}" ]] || continue
        if ! modinfo -k "${kver}" -F version nvidia >/dev/null 2>&1; then
            printf '%s\n' "${kver}"
        fi
    done < <(installed_kernel_versions)
}

nvidia_module_packages() {
    # nvidia_module_packages <driver-package>, e.g. nvidia-driver-615-open
    # Prints Ubuntu's prebuilt module packages for that driver: the kernel-flavour
    # metapackage (so future kernel updates pull in a matching nvidia.ko) plus the
    # per-kernel package of every kernel already installed
    local prefix="linux-modules-nvidia-${1#nvidia-driver-}"
    local item

    while IFS= read -r item; do
        [[ -n "${item}" ]] || continue
        if apt-cache show "${prefix}-${item}" >/dev/null 2>&1; then
            printf '%s\n' "${prefix}-${item}"
        fi
    done < <(installed_kernel_flavours; installed_kernel_versions)
}

install_nvidia_driver() {
    local driver_package
    local installed_branch
    local installed_version
    local -a missing_kernels
    local -a packages

    if [[ "${option_no_cuda_drivers}" == true ]]; then
        echo -e "\e[33mSkipping NVIDIA driver (--no-cuda-drivers).\e[m"
        return 0
    fi

    # The driver comes from Ubuntu's restricted archive with prebuilt, signed kernel
    # modules (no DKMS). Each kernel needs its own linux-modules-nvidia-* package, so
    # a kernel installed without one boots with no driver at all.
    driver_package="$(installed_nvidia_driver_package)"
    installed_branch="$(sed -E 's/^nvidia-driver-([0-9]+).*/\1/' <<<"${driver_package}")"

    if [[ -z "${driver_package}" ]]; then
        installed_version="$(detect_nvidia_driver_version)"
        if [[ "${installed_version%%.*}" =~ ^[0-9]+$ ]] && ((${installed_version%%.*} >= nvidia_driver_branch)); then
            echo -e "\e[33mNVIDIA driver ${installed_version} found but not from an Ubuntu nvidia-driver-* package; leaving it alone.\e[m"
            return 0
        fi
        driver_package="nvidia-driver-${nvidia_driver_branch}-open"
        echo -e "\e[36mNo NVIDIA driver package detected; installing ${driver_package}...\e[m"
    elif ((installed_branch < nvidia_driver_branch)); then
        echo -e "\e[36m${driver_package} is older than branch ${nvidia_driver_branch}; upgrading...\e[m"
        driver_package="nvidia-driver-${nvidia_driver_branch}-open"
    else
        mapfile -t missing_kernels < <(kernels_missing_nvidia_module)
        if ((${#missing_kernels[@]} == 0)); then
            echo -e "\e[32m${driver_package} installed with a module for every kernel, skipping.\e[m"
            return 0
        fi
        echo -e "\e[36m${driver_package} installed, but these kernels have no nvidia module: ${missing_kernels[*]}\e[m"
    fi

    mapfile -t packages < <(nvidia_module_packages "${driver_package}")
    apt_install install "${driver_package}" "${packages[@]}"

    mapfile -t missing_kernels < <(kernels_missing_nvidia_module)
    if ((${#missing_kernels[@]} > 0)); then
        echo -e "\e[33mWarning: no prebuilt nvidia module for kernel(s) ${missing_kernels[*]}; they will boot without the driver.\e[m"
        echo "Boot a different kernel from GRUB, or remove the kernel(s) above."
    fi

    echo -e "\e[32mDone.\e[0m ${driver_package} installed; reboot before running 'nvidia-smi'."
    echo "Use 'sudo apt full-upgrade' (not 'apt upgrade') so kernel updates also install their nvidia module."
}

install_cuda_toolkit() {
    local -a cuda_packages

    echo -e "\e[36mInstalling CUDA Toolkit ${cuda_version} with cuBLAS / NPP / cuFFT...\e[m"

    if [[ "${option_runtime}" == true ]]; then
        cuda_packages=(
            "cuda-runtime-${cuda_apt_suffix}"
            "cuda-cudart-${cuda_apt_suffix}"
            "libcublas-${cuda_apt_suffix}"
            "libcufft-${cuda_apt_suffix}"
            "libnpp-${cuda_apt_suffix}"
        )
    else
        cuda_packages=(
            "cuda-toolkit-${cuda_apt_suffix}"
            "cuda-cudart-${cuda_apt_suffix}"
            "cuda-cudart-dev-${cuda_apt_suffix}"
            "libcublas-${cuda_apt_suffix}"
            "libcublas-dev-${cuda_apt_suffix}"
            "libcufft-${cuda_apt_suffix}"
            "libcufft-dev-${cuda_apt_suffix}"
            "libnpp-${cuda_apt_suffix}"
            "libnpp-dev-${cuda_apt_suffix}"
        )
    fi

    apt_install install "${cuda_packages[@]}"

    # Superseded by the OpenCV env below once that build succeeds
    nvidia_env_content="export CUDA_HOME=\"/usr/local/cuda-${cuda_version}\"
export PATH=\"\${CUDA_HOME}/bin:\${PATH}\"
export LD_LIBRARY_PATH=\"\${CUDA_HOME}/lib64\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\""

    echo -e "\e[32mDone.\e[0m CUDA ${cuda_version} installed. Verify: /usr/local/cuda/bin/nvcc --version"
}

install_cudnn() {
    local -a cudnn_packages

    echo -e "\e[36mInstalling cuDNN 9 for CUDA ${cudnn_cuda_major}...\e[m"

    cudnn_packages=("libcudnn9-cuda-${cudnn_cuda_major}")
    if [[ "${option_runtime}" != true ]]; then
        cudnn_packages+=(
            "libcudnn9-dev-cuda-${cudnn_cuda_major}"
            "libcudnn9-headers-cuda-${cudnn_cuda_major}"
        )
    fi

    if ! apt-cache show "libcudnn9-cuda-${cudnn_cuda_major}" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping cuDNN: 'libcudnn9-cuda-${cudnn_cuda_major}' not found in any apt source.\e[m"
        echo "Install the cuDNN local repo from https://developer.nvidia.com/cudnn and re-run."
        return 0
    fi

    apt_install install "${cudnn_packages[@]}"
    echo -e "\e[32mDone.\e[0m cuDNN installed."
}

install_tensorrt() {
    local -a tensorrt_packages
    local trt_major="${tensorrt_version%%.*}"
    local pinned_version

    echo -e "\e[36mInstalling TensorRT ${tensorrt_version}...\e[m"

    if [[ "${option_runtime}" == true ]]; then
        tensorrt_packages=(
            "libnvinfer${trt_major}"
            "libnvinfer-plugin${trt_major}"
            "libnvonnxparsers${trt_major}"
        )
    else
        tensorrt_packages=(
            "libnvinfer-dev"
            "libnvinfer-plugin-dev"
            "libnvonnxparsers-dev"
            "libnvinfer-headers-dev"
            "libnvinfer-bin"
        )
    fi

    if ! apt-cache show "${tensorrt_packages[0]}" >/dev/null 2>&1; then
        echo -e "\e[33mSkipping TensorRT: '${tensorrt_packages[0]}' not found in any apt source.\e[m"
        echo "TensorRT needs the NVIDIA local apt repo (login required):"
        echo "  1. Download 'nv-tensorrt-local-repo-ubuntu2404-${tensorrt_version}-cuda-*.deb' from"
        echo "     https://developer.nvidia.com/tensorrt"
        echo "  2. sudo dpkg -i nv-tensorrt-local-repo-*.deb && sudo apt update"
        echo "  3. Re-run: ./setup-dev-env.sh --nvidia"
        return 0
    fi

    # Pin the whole TensorRT family to one release. Pinning only the requested packages
    # (pkg=version) is not enough: their exact-version dependencies (libnvinfer11, ...)
    # would still resolve to the newest candidate from the CUDA network repo (priority 600).
    # The preferences file also stops 'apt upgrade' from moving TensorRT to a newer release.
    # Prefer the build for the installed CUDA (+cuda13.4), else the newest CUDA 13 build
    pinned_version="$(apt_candidate_version "${tensorrt_packages[0]}" "${tensorrt_version}" "+${tensorrt_cuda_tag}")"
    if [[ -z "${pinned_version}" ]]; then
        pinned_version="$(apt_candidate_version "${tensorrt_packages[0]}" "${tensorrt_version}" "+cuda${cuda_version%%.*}.")"
    fi
    if [[ -n "${pinned_version}" ]]; then
        local pin_file="/etc/apt/preferences.d/asr-sdm-tensorrt-pin"
        local -a installed_trt_packages=()

        echo "Pinning TensorRT packages to ${pinned_version} (${pin_file})."
        echo "Running: sudo tee ${pin_file}"
        sudo tee "${pin_file}" >/dev/null <<EOF
# Written by asr_sdm_robo setup-dev-env.sh: keep every TensorRT package on ${tensorrt_version}
Package: libnvinfer* libnvonnxparsers* python3-libnvinfer* tensorrt*
Pin: version ${pinned_version}
Pin-Priority: 1001
EOF

        # Move TensorRT packages that are already installed (possibly a newer release) along too
        mapfile -t installed_trt_packages < <(
            dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n' \
                'libnvinfer*' 'libnvonnxparsers*' 'python3-libnvinfer*' 'tensorrt*' 2>/dev/null |
                awk '$2 == "ii" { print $1 }'
        )

        apt_install install --allow-downgrades "${tensorrt_packages[@]}" "${installed_trt_packages[@]}"
    else
        echo -e "\e[33mTensorRT ${tensorrt_version} not available in apt; installing the default candidate.\e[m"
        apt_install install "${tensorrt_packages[@]}"
    fi

    echo -e "\e[32mDone.\e[0m TensorRT installed. Verify: dpkg -l | grep -E 'libnvinfer|libnvonnx'"
}

find_cuda_lib() {
    # find_cuda_lib <soname-stem>, searching cuda_lib_dirs and skipping stub libraries
    local name="$1"
    local dir
    local candidate

    for dir in "${cuda_lib_dirs[@]}"; do
        for candidate in "${dir}/lib${name}.so" "${dir}/lib${name}.so."*; do
            if [[ -e "${candidate}" && "${candidate}" != *"/stubs/"* ]]; then
                readlink -f "${candidate}"
                return 0
            fi
        done
    done

    return 1
}

ensure_unversioned_cuda_lib() {
    # CMake's FindCUDA needs lib<name>.so next to the versioned library
    local name="$1"
    local real_path
    local lib_dir

    real_path="$(find_cuda_lib "${name}" || true)"
    if [[ -z "${real_path}" ]]; then
        return 0
    fi

    lib_dir="$(dirname "${real_path}")"
    if [[ -e "${lib_dir}/lib${name}.so" ]]; then
        return 0
    fi

    if [[ -w "${lib_dir}" ]]; then
        ln -sfn "$(basename "${real_path}")" "${lib_dir}/lib${name}.so"
    else
        sudo ln -sfn "$(basename "${real_path}")" "${lib_dir}/lib${name}.so"
    fi
}

install_opencv_cuda() {
    local install_prefix="${OPENCV_CUDA_PREFIX:-${HOME}/.local/opencv-cuda}"
    # Build next to the source in dependency/ (override with OPENCV_CUDA_BUILD_ROOT)
    local build_root="${OPENCV_CUDA_BUILD_ROOT:-${opencv_src_dir}}"
    local build_dir="${build_root}/build"
    local cuda_libs_root
    local cuda_lib_path
    local cuda_arch_bin
    local env_file
    local lib
    local lib_path
    local -a generator
    local -a missing_libs
    local -a cuda_lib_args
    # Every CUDA library OpenCV's FindCUDA looks up; the first four are required
    local -a cuda_libs=(cublas cufft nppc npps cublasLt nppial nppicc nppidei nppif nppig nppim nppist nppisu nppitc)

    # Only core + the contrib CUDA modules used by tensorrt_common
    local build_list="core,imgproc,imgcodecs,cudev,cudaarithm,cudawarping,cudaimgproc,cudafilters"

    echo -e "\e[36mSetting up OpenCV ${opencv_version} with CUDA...\e[m"

    apt_install install \
        build-essential \
        cmake \
        ninja-build \
        pkg-config \
        wget \
        unzip \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libtbb-dev

    # Prefer the versioned toolkit: /usr/local/cuda is an alternatives link that can
    # point at another (or a removed) CUDA release, and so can an inherited CUDA_HOME
    if [[ -n "${CUDA_HOME:-}" && ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
        echo -e "\e[33mIgnoring CUDA_HOME=${CUDA_HOME} (no nvcc there).\e[m"
        unset CUDA_HOME
    fi
    if [[ -x "/usr/local/cuda-${cuda_version}/bin/nvcc" ]]; then
        export CUDA_HOME="/usr/local/cuda-${cuda_version}"
    fi
    if [[ -z "${CUDA_HOME:-}" ]]; then
        if [[ -x /usr/local/cuda/bin/nvcc ]]; then
            export CUDA_HOME=/usr/local/cuda
        elif [[ -x "${HOME}/.local/cuda-${cuda_version}-root/usr/local/cuda-${cuda_version}/bin/nvcc" ]]; then
            export CUDA_HOME="${HOME}/.local/cuda-${cuda_version}-root/usr/local/cuda-${cuda_version}"
        fi
    fi

    if [[ -z "${CUDA_HOME:-}" || ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
        echo -e "\e[33mSkipping OpenCV: nvcc not found (install the CUDA Toolkit first, or set CUDA_HOME).\e[m"
        return 0
    fi

    export PATH="${CUDA_HOME}/bin:${PATH}"

    # cuBLAS / NPP / cuFFT may live in a separate tree from the toolkit
    cuda_libs_root="${CUDA_LIBS_ROOT:-${CUDA_HOME}}"
    cuda_lib_dirs=("${CUDA_HOME}/lib64")
    if [[ -d "${cuda_libs_root}/targets/x86_64-linux/lib" ]]; then
        cuda_lib_dirs+=("${cuda_libs_root}/targets/x86_64-linux/lib")
    fi
    cuda_lib_path="$(IFS=:; echo "${cuda_lib_dirs[*]}")"
    export LD_LIBRARY_PATH="${cuda_lib_path}:${LD_LIBRARY_PATH:-}"
    export LIBRARY_PATH="${cuda_lib_path}:${LIBRARY_PATH:-}"

    missing_libs=()
    cuda_lib_args=()
    for lib in "${cuda_libs[@]}"; do
        ensure_unversioned_cuda_lib "${lib}" || true
        lib_path="$(find_cuda_lib "${lib}" || true)"
        if [[ -n "${lib_path}" ]]; then
            # cublasLt is linked through cublas, FindCUDA has no variable for it
            [[ "${lib}" == "cublasLt" ]] || cuda_lib_args+=(-D "CUDA_${lib}_LIBRARY=${lib_path}")
        elif [[ "${lib}" =~ ^(cublas|cufft|nppc|npps)$ ]]; then
            missing_libs+=("${lib}")
        fi
    done

    if ((${#missing_libs[@]})); then
        echo -e "\e[33mSkipping OpenCV: missing CUDA libraries: ${missing_libs[*]}\e[m"
        echo "Expected under ${cuda_lib_path}"
        echo "Install libcublas-${cuda_apt_suffix} / libcufft-${cuda_apt_suffix} / libnpp-${cuda_apt_suffix}, or set CUDA_LIBS_ROOT."
        return 0
    fi

    cuda_arch_bin="${OPENCV_CUDA_ARCH_BIN:-}"
    if [[ -z "${cuda_arch_bin}" ]]; then
        cuda_arch_bin="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ' || true)"
        # nvidia-smi prints errors on stdout (e.g. "Driver/library version mismatch"
        # right after a driver install, before the reboot), so accept only "X.Y"
        if [[ ! "${cuda_arch_bin}" =~ ^[0-9]+\.[0-9]+$ ]]; then
            echo -e "\e[33mCould not read the GPU compute capability from nvidia-smi; using 8.6 (set OPENCV_CUDA_ARCH_BIN to override).\e[m"
            cuda_arch_bin="8.6"
        fi
    fi

    echo "  CUDA_HOME       = ${CUDA_HOME}"
    echo "  CUDA_LIBS_ROOT  = ${cuda_libs_root}"
    echo "  CUDA_ARCH_BIN   = ${cuda_arch_bin}"
    echo "  SOURCE_DIR      = ${opencv_src_dir}"
    echo "  CONTRIB_DIR     = ${opencv_contrib_src_dir}"
    echo "  INSTALL_PREFIX  = ${install_prefix}"
    echo "  BUILD_DIR       = ${build_dir}"

    if [[ "${option_rebuild}" != true \
        && -f "${install_prefix}/include/opencv4/opencv2/cudaimgproc.hpp" ]] \
        && grep -qs "OpenCV_VERSION ${opencv_version})" "${install_prefix}/lib/cmake/opencv4/OpenCVConfig-version.cmake"; then
        echo "OpenCV ${opencv_version} with CUDA already installed in ${install_prefix}, skipping build (use --rebuild to force)."
    else
        build_opencv_cuda
    fi

    write_opencv_env
}

# Called from install_opencv_cuda; reads its locals (install_prefix, build_dir,
# cuda_* paths, cuda_lib_args, build_list) through bash dynamic scoping.
build_opencv_cuda() {
    mkdir -p "${install_prefix}"
    ensure_opencv_source

    rm -rf "${build_dir}"
    mkdir -p "${build_dir}"
    mapfile -t generator < <(cmake_generator_args)

    reset_stale_cmake_cache "${opencv_src_dir}" "${build_dir}"

    run_cmd cmake -S "${opencv_src_dir}" -B "${build_dir}" "${generator[@]}" \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX="${install_prefix}" \
        -D CMAKE_CXX_STANDARD=17 \
        -D CMAKE_PREFIX_PATH="${cuda_libs_root}/targets/x86_64-linux" \
        -D CMAKE_LIBRARY_PATH="${cuda_lib_path}" \
        -D OPENCV_EXTRA_MODULES_PATH="${opencv_contrib_src_dir}/modules" \
        -D BUILD_LIST="${build_list}" \
        -D WITH_CUDA=ON \
        -D WITH_CUDNN=ON \
        -D OPENCV_DNN_CUDA=OFF \
        -D ENABLE_FAST_MATH=ON \
        -D CUDA_FAST_MATH=ON \
        -D WITH_CUBLAS=ON \
        -D WITH_CUFFT=ON \
        -D CUDA_ARCH_BIN="${cuda_arch_bin}" \
        -D CUDA_ARCH_PTX="" \
        -D CUDA_TOOLKIT_ROOT_DIR="${CUDA_HOME}" \
        -D CUDA_NVCC_FLAGS="-std=c++17" \
        -D CMAKE_CUDA_STANDARD=17 \
        -D CMAKE_CXX_FLAGS="-I${cuda_libs_root}/targets/x86_64-linux/include" \
        "${cuda_lib_args[@]}" \
        -D CUDNN_INCLUDE_DIR=/usr/include/x86_64-linux-gnu \
        -D CUDNN_LIBRARY=/usr/lib/x86_64-linux-gnu/libcudnn.so \
        -D WITH_GTK=OFF \
        -D WITH_QT=OFF \
        -D WITH_OPENGL=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_opencv_apps=OFF \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=OFF \
        -D BUILD_JAVA=OFF \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_ENABLE_NONFREE=OFF

    echo "Building (this can take a while)..."
    run_cmd cmake --build "${build_dir}" --parallel "$(nproc)"
    run_cmd cmake --install "${build_dir}"

    if [[ ! -f "${install_prefix}/include/opencv4/opencv2/cudaimgproc.hpp" ]]; then
        echo -e "\e[33mWarning: cudaimgproc.hpp is missing; the OpenCV CUDA modules were not built.\e[m"
    fi
    echo "Source tree: ${opencv_src_dir}"
}

# Called from install_opencv_cuda, like build_opencv_cuda above.
write_opencv_env() {
    # Same content lands in ~/.bashrc and in the sourceable env file, with $HOME left unexpanded
    local prefix_expr cuda_home_expr cuda_lib_path_expr
    prefix_expr="$(portable_home_path "${install_prefix}")"
    cuda_home_expr="$(portable_home_path "${CUDA_HOME}")"
    cuda_lib_path_expr="$(portable_home_path_list "${cuda_lib_path}")"

    nvidia_env_content="export CUDA_HOME=\"${cuda_home_expr}\"
export PATH=\"\${CUDA_HOME}/bin:\${PATH}\"
export LD_LIBRARY_PATH=\"${prefix_expr}/lib:${cuda_lib_path_expr}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\"
export OpenCV_DIR=\"${prefix_expr}/lib/cmake/opencv4\"
export PKG_CONFIG_PATH=\"${prefix_expr}/lib/pkgconfig\${PKG_CONFIG_PATH:+:\${PKG_CONFIG_PATH}}\""

    env_file="${install_prefix}/setup_env.sh"
    {
        echo "# Generated by setup-dev-env.sh --nvidia"
        echo "${nvidia_env_content}"
    } > "${env_file}"

    echo -e "\e[32mDone.\e[0m OpenCV ${opencv_version} installed into ${install_prefix}."
    echo "Sourceable copy of the same env: ${env_file}"
}

installed_nvidia_packages() {
    # Prints every installed (or removed-but-not-purged) package of the NVIDIA stack:
    # driver, kernel modules, CUDA, cuDNN, TensorRT, NCCL and the NVIDIA apt repo packages
    dpkg-query -W -f='${Package} ${db:Status-Abbrev}\n' \
        '*nvidia*' 'cuda*' 'libcuda*' 'libcudnn*' 'libcublas*' 'libcufft*' 'libcufile*' \
        'libcurand*' 'libcusolver*' 'libcusparse*' 'libcupti*' 'libnpp*' 'libnvrtc*' \
        'libnvjitlink*' 'libnvjpeg*' 'libnvvm*' 'libnvtoolsext*' 'libnvblas*' 'libnccl*' \
        'libnvfatbin*' 'libcuobjclient*' \
        'libnvinfer*' 'libnvonnxparsers*' 'libnvparsers*' 'python3-libnvinfer*' 'tensorrt*' \
        'nv-tensorrt-local-repo-*' 'cudnn-local-repo-*' 'nsight-*' 2>/dev/null |
        awk '$2 ~ /^.[^n]/ { print $1 }' |
        # linux-firmware-* belongs to the generic linux-firmware set (also used by nouveau), and
        # removing it would drag linux-firmware and the kernel metapackages along
        grep -v '^linux-firmware' | sort -u || true   # dpkg-query fails when nothing matches
    # libxnvctrl0 is deliberately not matched: libhwloc-plugins depends on it, so purging it would
    # also remove OpenMPI, PCL, VTK, libopencv-dev and ros-jazzy-desktop. autoremove drops it when unused.
}

purge_nvidia_stack() {
    local opencv_prefix="${OPENCV_CUDA_PREFIX:-${HOME}/.local/opencv-cuda}"
    local tensorrt_pin_file="/etc/apt/preferences.d/asr-sdm-tensorrt-pin"
    local -a packages
    local -a collateral

    echo -e "\e[36mPurging the NVIDIA stack (driver, CUDA, cuDNN, TensorRT, OpenCV + CUDA)...\e[m"

    mapfile -t packages < <(installed_nvidia_packages)
    if ((${#packages[@]} > 0)); then
        echo "Packages to purge (${#packages[@]}):"
        printf '  %s\n' "${packages[@]}"

        # Refuse when apt would also remove packages outside the NVIDIA stack (reverse dependencies)
        mapfile -t collateral < <(
            comm -23 \
                <(apt-get -s purge "${packages[@]}" 2>/dev/null | awk '$1 == "Purg" { print $2 }' | sort -u) \
                <(printf '%s\n' "${packages[@]}")
        )
        if ((${#collateral[@]} > 0)); then
            echo -e "\e[31mPurging these would also remove packages outside the NVIDIA stack:\e[m"
            printf '  %s\n' "${collateral[@]}"
            if [[ "${option_yes}" == true ]]; then
                echo "Aborting (-y). Purge them by hand, or remove the packages that depend on them first."
                return 1
            fi
            echo "Review apt's list below carefully before confirming."
        fi

        apt_install purge "${packages[@]}"
        apt_install autoremove --purge
        # Purging cuda-keyring / the local repo packages drops their apt sources
        mark_apt_sources_changed
    else
        echo "No NVIDIA packages installed."
    fi

    if [[ -e "${tensorrt_pin_file}" ]]; then
        run_cmd sudo rm -f "${tensorrt_pin_file}"
    fi

    # OpenCV built against the purged CUDA cannot load anymore
    if [[ -d "${opencv_prefix}" ]]; then
        run_cmd rm -rf "${opencv_prefix}"
    fi

    if remove_bashrc_block "asr_sdm nvidia env"; then
        echo "Removed 'asr_sdm nvidia env' from ~/.bashrc."
    fi

    echo -e "\e[32mDone.\e[0m NVIDIA stack purged; reboot to unload the driver."
    echo "Sources in ${dependency_dir} are kept (delete opencv-* there to free space)."
}

install_nvidia_stack() {
    if [[ "${option_no_nvidia}" == true ]]; then
        echo -e "\e[33mSkipping the NVIDIA stack (--no-nvidia).\e[m"
        return 0
    fi

    if ! check_nvidia_prerequisites; then
        return 0
    fi

    setup_cuda_apt_repo
    install_nvidia_driver
    install_cuda_toolkit
    install_cudnn
    install_tensorrt
    install_opencv_cuda

    if [[ -n "${nvidia_env_content}" ]]; then
        write_bashrc_block "asr_sdm nvidia env" "${nvidia_env_content}"
    fi

    echo -e "\e[32mNVIDIA stack setup complete.\e[0m"
    echo "Reload: source ~/.bashrc  |  Reboot first if the driver was installed."
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h | --help)
                print_help
                exit 0
                ;;
            --list)
                option_list=true
                shift
                ;;
            --common)
                option_common=true
                shift
                ;;
            --ros)
                option_ros=true
                shift
                ;;
            --dependency)
                # Removed: c-periphery is now part of --common
                echo -e "\e[33mWarning: --dependency was removed; c-periphery is installed by --common. Running --common.\e[m"
                option_common=true
                shift
                ;;
            --nvidia)
                option_nvidia=true
                shift
                ;;
            --purge-nvidia)
                option_purge_nvidia=true
                shift
                ;;
            --no-nvidia)
                option_no_nvidia=true
                shift
                ;;
            --no-cuda-drivers)
                option_no_cuda_drivers=true
                shift
                ;;
            --runtime)
                option_runtime=true
                shift
                ;;
            --rebuild)
                option_rebuild=true
                shift
                ;;
            -y)
                option_yes=true
                shift
                ;;
            -v | --download-artifacts)
                echo "Warning: option '$1' is not implemented yet, ignoring."
                shift
                ;;
            --ros-distro)
                if [[ $# -lt 2 ]]; then
                    echo "Error: option '$1' requires an argument."
                    print_help
                    exit 1
                fi
                if [[ "$2" != "${ros_distro}" ]]; then
                    echo "Error: unsupported ROS distribution '$2'. Only '${ros_distro}' is supported."
                    exit 1
                fi
                shift 2
                ;;
            --data-dir | --module)
                if [[ $# -lt 2 ]]; then
                    echo "Error: option '$1' requires an argument."
                    print_help
                    exit 1
                fi
                echo "Warning: option '$1' is not implemented yet, ignoring."
                shift 2
                ;;
            *)
                echo "Error: unknown option '$1'"
                print_help
                exit 1
                ;;
        esac
    done
}

main() {
    parse_args "$@"

    if [[ "${option_list}" == true ]]; then
        print_list
        exit 0
    fi

    if [[ "${option_common}" != true && "${option_ros}" != true && "${option_nvidia}" != true \
        && "${option_purge_nvidia}" != true ]]; then
        print_help
        echo "No actionable option provided. Use --list, --common, --ros, --nvidia and/or --purge-nvidia."
        exit 1
    fi

    confirm_or_exit
    ensure_sudo

    # Runs on success or failure, after any step that wrote into dependency/ as root
    trap 'fix_dependency_owner || true' EXIT
    migrate_legacy_dependency_dir

    # Purge before any install so --purge-nvidia --nvidia gives a clean reinstall
    if [[ "${option_purge_nvidia}" == true ]]; then
        purge_nvidia_stack
    fi

    if [[ "${option_common}" == true ]]; then
        install_common
    fi

    if [[ "${option_ros}" == true ]]; then
        install_ros
    fi

    if [[ "${option_nvidia}" == true ]]; then
        install_nvidia_stack
    fi
}

main "$@"
