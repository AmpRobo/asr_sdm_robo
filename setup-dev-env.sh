#!/usr/bin/env bash
# Set up development environment for asr_sdm_robo.
# Usage: ./setup-dev-env.sh [OPTIONS]
# Note: -y option is only for CI.

set -euo pipefail

option_pinocchio=false
option_ros=false
option_yes=false
option_list=false

print_help() {
    echo "Usage: setup-dev-env.sh [OPTIONS]"
    echo "Options:"
    echo "  --help                  Display this help message"
    echo "  -h                      Display this help message"
    echo "  --list                  List all installable components"
    echo "  --ros                   Install ROS 2 Jazzy (Ubuntu 24.04)"
    echo "  --pinocchio             Install Pinocchio from robotpkg apt packages"
    echo "  -y                      Use non-interactive mode"
    echo "  -v                      Enable debug outputs"
    echo "  --no-nvidia             Disable installation of the NVIDIA-related roles ('cuda' and 'tensorrt')"
    echo "  --no-cuda-drivers       Disable installation of 'cuda-drivers' in the role 'cuda'"
    echo "  --runtime               Disable installation dev package of role 'cuda' and 'tensorrt'"
    echo "  --data-dir              Set data directory (default: $HOME/asr_sdm_data)"
    echo "  --download-artifacts    Download artifacts"
    echo "  --module                Specify the module (default: all)"
    echo "  --ros-distro            Specify ROS distribution (rolling or jazzy, default: jazzy)"
    echo ""
}

print_list() {
    echo "Installable components:"
    echo ""
    echo "  Flag          Component    Description"
    echo "  ----          ---------    -----------"
    echo "  --ros         ROS 2        Install ROS 2 Jazzy desktop + rosdep/colcon (Ubuntu 24.04)"
    echo "  --pinocchio   Pinocchio    Install Pinocchio and Python bindings via robotpkg apt"
    echo ""
    echo "Examples:"
    echo "  ./setup-dev-env.sh --list"
    echo "  ./setup-dev-env.sh --ros"
    echo "  ./setup-dev-env.sh --pinocchio"
    echo "  ./setup-dev-env.sh --ros --pinocchio -y"
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

    echo -e "\e[33mThis will install the selected development packages.\e[m"
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

install_ros() {
    local ubuntu_version
    local ros_version=""

    if ! command -v sudo >/dev/null 2>&1; then
        apt-get -y update
        apt-get -y install sudo
    fi

    apt_install update
    apt_install install \
        git \
        curl \
        ca-certificates \
        build-essential \
        pkg-config \
        cmake \
        python3 \
        python3-pip \
        python3-venv

    ubuntu_version="$(lsb_release -sc)"
    case "${ubuntu_version}" in
        noble)
            ros_version="jazzy"
            ;;
        *)
            echo -e "\e[33mSkipping ROS 2: unsupported Ubuntu (${ubuntu_version}). Supported: noble (24.04).\e[m"
            return 0
            ;;
    esac

    echo -e "\e[36mInstalling ROS 2 ${ros_version}...\e[m"

    apt_install install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    apt_install install software-properties-common
    sudo add-apt-repository -y universe
    apt_install update

    local ros_apt_source_version
    ros_apt_source_version="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')"
    curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb

    apt_install update
    apt_install install ros-dev-tools
    apt_install update
    apt_install upgrade
    apt_install install "ros-${ros_version}-desktop"

    apt_install install python3-rosdep python3-colcon-common-extensions
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        sudo rosdep init
    fi
    rosdep update

    if ! grep -q "ROS 2 ${ros_version}" ~/.bashrc 2>/dev/null; then
        {
            echo ""
            echo "# ROS 2 ${ros_version} environment"
            echo "source /opt/ros/${ros_version}/setup.bash"
            echo "source /usr/share/colcon_cd/function/colcon_cd.sh"
            echo "export _colcon_cd_root=/opt/ros/${ros_version}/"
        } >> ~/.bashrc
    fi

    echo -e "\e[32mDone.\e[0m ROS 2 ${ros_version} installed."
    echo "Reload: source ~/.bashrc  |  Verify: ros2 run demo_nodes_cpp talker"
}

install_pinocchio_from_apt() {
    local robotpkg_key_url="http://robotpkg.openrobots.org/packages/debian/robotpkg.asc"
    local robotpkg_key_path="/etc/apt/keyrings/robotpkg.asc"
    local robotpkg_list_path="/etc/apt/sources.list.d/robotpkg.list"
    local bashrc_path="${HOME}/.bashrc"
    local python_version
    local python_version_short
    local codename
    local repo_line
    local -a pinocchio_packages

    python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    python_version_short="$(python3 -c 'import sys; print(f"{sys.version_info.major}{sys.version_info.minor}")')"

    pinocchio_packages=(
        "robotpkg-pinocchio"
        "robotpkg-py${python_version_short}-pinocchio"
        "robotpkg-coal"
        "robotpkg-py${python_version_short}-coal"
        "robotpkg-py${python_version_short}-eigenpy"
    )

    run_cmd sudo apt update
    run_cmd sudo apt install -qqy lsb-release curl

    run_cmd sudo mkdir -p /etc/apt/keyrings
    run_cmd sudo curl -fsSL "${robotpkg_key_url}" -o "${robotpkg_key_path}"

    codename="$(lsb_release -cs)"
    repo_line="deb [arch=amd64 signed-by=${robotpkg_key_path}] http://robotpkg.openrobots.org/packages/debian/pub ${codename} robotpkg"

    if [[ -f "${robotpkg_list_path}" ]] && grep -qxF "${repo_line}" "${robotpkg_list_path}"; then
        echo "robotpkg apt source already exists, skipping."
    else
        echo "Running: sudo tee -a ${robotpkg_list_path}"
        echo "${repo_line}" | sudo tee -a "${robotpkg_list_path}" >/dev/null
    fi

    run_cmd sudo apt update
    run_cmd sudo apt install --reinstall -qqy "${pinocchio_packages[@]}"

    if [[ -f "${bashrc_path}" ]] && grep -q "pinocchio robotpkg setup" "${bashrc_path}"; then
        echo "~/.bashrc already contains Pinocchio environment variables, skipping."
    else
        cat >> "${bashrc_path}" <<EOF

# >>> pinocchio robotpkg setup >>>
export PATH=/opt/openrobots/bin:\$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python${python_version}/site-packages:\$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH
# <<< pinocchio robotpkg setup <<<
EOF
        echo "Appended Pinocchio environment variables to ~/.bashrc. Reload or reopen the terminal."
    fi

    echo "Pinocchio installation and environment setup complete."
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
            --ros)
                option_ros=true
                shift
                ;;
            --pinocchio)
                option_pinocchio=true
                shift
                ;;
            -y)
                option_yes=true
                shift
                ;;
            -v | --no-nvidia | --no-cuda-drivers | --runtime | --download-artifacts)
                echo "Warning: option '$1' is not implemented yet, ignoring."
                shift
                ;;
            --data-dir | --module | --ros-distro)
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

    if [[ "${option_ros}" != true && "${option_pinocchio}" != true ]]; then
        print_help
        echo "No actionable option provided. Use --list, --ros and/or --pinocchio."
        exit 1
    fi

    confirm_or_exit

    if [[ "${option_ros}" == true ]]; then
        install_ros
    fi

    if [[ "${option_pinocchio}" == true ]]; then
        install_pinocchio_from_apt
    fi
}

main "$@"
