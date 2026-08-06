#!/usr/bin/env bash
#
# build_without_perception.sh
#
# Purpose:
#   Build all colcon packages in the workspace except those under perception.
#   Automatically scans src/asr_sdm_universe/perception/ and skips them via
#   colcon --packages-skip.
#
# Notes:
#   - Perception message packages under asr_sdm_msgs are not skipped
#   - Passes --symlink-install by default
#   - Run from the workspace root after sourcing the ROS environment
#
# Usage:
#   ./build_without_perception.sh [options] [colcon-args...]
#
# Options:
#   --list-only           List packages that would be skipped, then exit
#   --no-symlink-install  Do not pass --symlink-install
#   -h, --help            Show help
#
# Examples:
#   ./build_without_perception.sh
#   ./build_without_perception.sh --list-only
#   ./build_without_perception.sh --parallel-workers 4
#   ./build_without_perception.sh --cmake-args -DCMAKE_BUILD_TYPE=Release
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${SCRIPT_DIR}"
PERCEPTION_DIR="${WORKSPACE_ROOT}/src/asr_sdm_universe/perception"

LIST_ONLY=0
SYMLINK_INSTALL=1
COLCON_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./build_without_perception.sh [options] [colcon-args...]

Build the workspace while skipping all packages under
src/asr_sdm_universe/perception/.

Options:
  --list-only           Print packages that would be skipped and exit
  --no-symlink-install  Do not pass --symlink-install to colcon
  -h, --help            Show this help

Examples:
  ./build_without_perception.sh
  ./build_without_perception.sh --parallel-workers 4
  ./build_without_perception.sh --cmake-args -DCMAKE_BUILD_TYPE=Release
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

if [[ ! -d "${PERCEPTION_DIR}" ]]; then
  echo "Perception directory not found: ${PERCEPTION_DIR}" >&2
  exit 1
fi

mapfile -t SKIP_PACKAGES < <(
  find "${PERCEPTION_DIR}" -name package.xml -print0 \
    | xargs -0 grep -h '<name>' \
    | sed -E 's/.*<name>([^<]+)<\/name>.*/\1/' \
    | sed 's/^[[:space:]]*//;s/[[:space:]]*$//' \
    | sort -u
)

if [[ ${#SKIP_PACKAGES[@]} -eq 0 ]]; then
  echo "No packages found under ${PERCEPTION_DIR}" >&2
  exit 1
fi

if [[ "${LIST_ONLY}" -eq 1 ]]; then
  echo "Would skip ${#SKIP_PACKAGES[@]} perception package(s):"
  printf '  %s\n' "${SKIP_PACKAGES[@]}"
  exit 0
fi

CMD=(colcon build)
if [[ "${SYMLINK_INSTALL}" -eq 1 ]]; then
  CMD+=(--symlink-install)
fi
CMD+=(--packages-skip "${SKIP_PACKAGES[@]}")
if [[ ${#COLCON_ARGS[@]} -gt 0 ]]; then
  CMD+=("${COLCON_ARGS[@]}")
fi

echo "Skipping ${#SKIP_PACKAGES[@]} perception package(s)"
printf '%q ' "${CMD[@]}"
echo
cd "${WORKSPACE_ROOT}"
exec "${CMD[@]}"
