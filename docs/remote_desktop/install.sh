#!/usr/bin/env bash
# Installs (or with --uninstall removes) the GNOME Remote Desktop workaround that
# runs the remote session's gnome-shell on Mesa EGL. See README.md in this folder.
# Usage: ./install.sh [--uninstall]
# Per-user only, no sudo. Takes effect after logging out of the remote session.

set -euo pipefail

src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
wrapper="${HOME}/.local/bin/gnome-shell-remote-egl"
dropin_dir="${HOME}/.config/systemd/user/org.gnome.Shell@wayland.service.d"
dropin="${dropin_dir}/remote-mesa-egl.conf"
mesa_vendor="/usr/share/glvnd/egl_vendor.d/50_mesa.json"

case "${1:-}" in
    "")
        if [[ ! -f "${mesa_vendor}" ]]; then
            echo "Error: ${mesa_vendor} not found (install libegl-mesa0)." >&2
            exit 1
        fi
        install -D -m 755 "${src_dir}/gnome-shell-remote-egl" "${wrapper}"
        install -D -m 644 "${src_dir}/remote-mesa-egl.conf" "${dropin}"
        echo "Installed ${wrapper}"
        echo "Installed ${dropin}"
        ;;
    --uninstall)
        rm -f "${wrapper}" "${dropin}"
        rmdir --ignore-fail-on-non-empty "${dropin_dir}" 2>/dev/null || true
        echo "Removed ${wrapper} and ${dropin}"
        ;;
    *)
        echo "Usage: $0 [--uninstall]" >&2
        exit 1
        ;;
esac

systemctl --user daemon-reload
echo "Log out of the remote session (top-right menu > Log Out) and log in again to apply."
echo "Closing the RDP window is not enough: the session keeps running."
