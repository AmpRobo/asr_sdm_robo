#!/usr/bin/env bash
#
# Install the ONNX Runtime C++ SDK (prebuilt linux binary) and export
# ONNXRUNTIME_ROOT in ~/.bashrc for asr_sdm_video_enhancement_ml.
#
# Usage:
#   ./install_onnxruntime.sh [version]
#   ONNXRUNTIME_VERSION=1.27.1 ./install_onnxruntime.sh
#
# Defaults:
#   version: 1.27.1
#   install root: $HOME/.local/onnxruntime
#
set -euo pipefail

VERSION="${1:-${ONNXRUNTIME_VERSION:-1.27.1}}"
INSTALL_BASE="${ONNXRUNTIME_INSTALL_BASE:-$HOME/.local/onnxruntime}"
BASHRC="${HOME}/.bashrc"

ARCH="$(uname -m)"
case "${ARCH}" in
  x86_64)  PLATFORM="linux-x64" ;;
  aarch64) PLATFORM="linux-aarch64" ;;
  *)
    echo "Unsupported architecture: ${ARCH}" >&2
    exit 1
    ;;
esac

ARCHIVE="onnxruntime-${PLATFORM}-${VERSION}.tgz"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${ARCHIVE}"
DEST="${INSTALL_BASE}/onnxruntime-${PLATFORM}-${VERSION}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

echo "=== ONNX Runtime C++ SDK ${VERSION} (${PLATFORM}) ==="
echo "  Download: ${URL}"
echo "  Install:  ${DEST}"

if [[ -f "${DEST}/include/onnxruntime_cxx_api.h" && -f "${DEST}/lib/libonnxruntime.so" ]]; then
  echo "=== Already installed at ${DEST}, skipping download ==="
else
  mkdir -p "${INSTALL_BASE}"
  echo "=== Downloading ${ARCHIVE} ==="
  if command -v curl >/dev/null 2>&1; then
    curl -fL --progress-bar -o "${TMP_DIR}/${ARCHIVE}" "${URL}"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "${TMP_DIR}/${ARCHIVE}" "${URL}"
  else
    echo "Need curl or wget to download ONNX Runtime." >&2
    exit 1
  fi

  echo "=== Extracting to ${INSTALL_BASE} ==="
  tar -xzf "${TMP_DIR}/${ARCHIVE}" -C "${INSTALL_BASE}"

  if [[ ! -f "${DEST}/include/onnxruntime_cxx_api.h" ]]; then
    echo "Extraction failed: headers not found at ${DEST}/include" >&2
    exit 1
  fi
  if [[ ! -f "${DEST}/lib/libonnxruntime.so" ]]; then
    echo "Extraction failed: library not found at ${DEST}/lib" >&2
    exit 1
  fi
fi

EXPORT_LINE="export ONNXRUNTIME_ROOT=\"${DEST}\""
MARKER_BEGIN="# >>> onnxruntime (asr_sdm_robo) >>>"
MARKER_END="# <<< onnxruntime (asr_sdm_robo) <<<"
BLOCK=$(cat <<EOF
${MARKER_BEGIN}
${EXPORT_LINE}
${MARKER_END}
EOF
)

touch "${BASHRC}"
if grep -Fq "${MARKER_BEGIN}" "${BASHRC}"; then
  echo "=== Updating ONNXRUNTIME_ROOT block in ${BASHRC} ==="
  # Replace existing managed block in-place.
  tmp_bashrc="$(mktemp)"
  awk -v begin="${MARKER_BEGIN}" -v end="${MARKER_END}" -v block="${BLOCK}" '
    $0 == begin { print block; skip=1; next }
    skip && $0 == end { skip=0; next }
    !skip { print }
  ' "${BASHRC}" > "${tmp_bashrc}"
  mv "${tmp_bashrc}" "${BASHRC}"
elif grep -Eq '^[[:space:]]*export[[:space:]]+ONNXRUNTIME_ROOT=' "${BASHRC}"; then
  echo "=== Replacing existing ONNXRUNTIME_ROOT export in ${BASHRC} ==="
  tmp_bashrc="$(mktemp)"
  grep -Ev '^[[:space:]]*export[[:space:]]+ONNXRUNTIME_ROOT=' "${BASHRC}" > "${tmp_bashrc}"
  printf '\n%s\n' "${BLOCK}" >> "${tmp_bashrc}"
  mv "${tmp_bashrc}" "${BASHRC}"
else
  echo "=== Appending ONNXRUNTIME_ROOT export to ${BASHRC} ==="
  printf '\n%s\n' "${BLOCK}" >> "${BASHRC}"
fi

echo "=== ONNX Runtime C++ SDK ${VERSION} installed successfully ==="
echo "  ONNXRUNTIME_ROOT=${DEST}"
echo "  Headers:  ${DEST}/include/"
echo "  Library:  ${DEST}/lib/libonnxruntime.so"
echo
echo "Load it in this shell with:"
echo "  source ~/.bashrc"
echo "or:"
echo "  export ONNXRUNTIME_ROOT=\"${DEST}\""
echo
echo "Then build:"
echo "  colcon build --packages-select asr_sdm_video_enhancement_ml"
