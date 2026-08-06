#!/usr/bin/env bash
#
# build_video_inertial_navigation_systems.sh
#
# Purpose:
#   Build only packages under asr_sdm_video_inertial_navigation_systems.
#   Automatically scans that directory and selects them via
#   colcon --packages-select.
#
# Target directory:
#   src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/
#
# Packages included (may change; use --list-only for the current list):
#   ar_demo, benchmark_publisher, camera_model, config_pkg,
#   feature_tracker, pose_graph, vins_estimator
#
# Notes:
#   - Passes --symlink-install by default
#   - Builds only the selected packages, not unmet dependencies
#   - Run from the workspace root after sourcing the ROS environment
#
# Usage:
#   ./build_video_inertial_navigation_systems.sh [options] [colcon-args...]
#
# Options:
#   --list-only           List packages that would be built, then exit
#   --no-symlink-install  Do not pass --symlink-install
#   -h, --help            Show help
#
# Examples:
#   ./build_video_inertial_navigation_systems.sh
#   ./build_video_inertial_navigation_systems.sh --list-only
#   ./build_video_inertial_navigation_systems.sh --parallel-workers 4
#   ./build_video_inertial_navigation_systems.sh --cmake-args -DCMAKE_BUILD_TYPE=Release
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
TARGET_DIR="${WORKSPACE_ROOT}/src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems"

LIST_ONLY=0
SYMLINK_INSTALL=1
COLCON_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./build_video_inertial_navigation_systems.sh [options] [colcon-args...]

Build only packages under
src/asr_sdm_universe/perception/asr_sdm_video_inertial_navigation_systems/.

Options:
  --list-only           Print packages that would be built and exit
  --no-symlink-install  Do not pass --symlink-install to colcon
  -h, --help            Show this help

Examples:
  ./build_video_inertial_navigation_systems.sh
  ./build_video_inertial_navigation_systems.sh --parallel-workers 4
  ./build_video_inertial_navigation_systems.sh --cmake-args -DCMAKE_BUILD_TYPE=Release
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --list-only)
      LIST_ONLY=1
      shift
      ;;
    --no-symlink-install)
      SYMLINK_INSTALL=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      COLCON_ARGS+=("$@")
      break
      ;;
  esac
done

if [[ ! -d "${TARGET_DIR}" ]]; then
  echo "Target directory not found: ${TARGET_DIR}" >&2
  exit 1
fi

mapfile -t SELECT_PACKAGES < <(
  find "${TARGET_DIR}" -name package.xml -print0 \
    | xargs -0 grep -h '<name>' \
    | sed -E 's/.*<name>([^<]+)<\/name>.*/\1/' \
    | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' \
    | sort -u
)

if [[ ${#SELECT_PACKAGES[@]} -eq 0 ]]; then
  echo "No packages found under ${TARGET_DIR}" >&2
  exit 1
fi

if [[ "${LIST_ONLY}" -eq 1 ]]; then
  echo "Would build ${#SELECT_PACKAGES[@]} package(s):"
  printf '  %s\n' "${SELECT_PACKAGES[@]}"
  exit 0
fi

CMD=(colcon build)
if [[ "${SYMLINK_INSTALL}" -eq 1 ]]; then
  CMD+=(--symlink-install)
fi
CMD+=(--packages-select "${SELECT_PACKAGES[@]}")
if [[ ${#COLCON_ARGS[@]} -gt 0 ]]; then
  CMD+=("${COLCON_ARGS[@]}")
fi

echo "Building ${#SELECT_PACKAGES[@]} package(s) from asr_sdm_video_inertial_navigation_systems"
printf '%q ' "${CMD[@]}"
echo
cd "${WORKSPACE_ROOT}"
exec "${CMD[@]}"
