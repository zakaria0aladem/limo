#!/usr/bin/env bash
# ============================================================================
# install_mocap_localization.sh
# ----------------------------------------------------------------------------
# Installs the system deps and builds mocap_localization INSIDE the Foxy
# container. Run it from wherever the package sits in the mounted workspace:
#
#   # host:       cp -r ~/limo/src/mocap_localization ~/ros2_ws/src/
#   # container:  bash /root/ros2_ws/src/mocap_localization/install_mocap_localization.sh
#
# If the script is run from a package copy OUTSIDE $WS/src (and that path is
# visible in the container), it copies the package into $WS/src first. It
# copies rather than symlinks: a link to a host path the container doesn't
# mount would dangle.
# ============================================================================
set -euo pipefail

WS="${ROS2_WS:-/root/ros2_ws}"
SRC="$WS/src"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "$PKG_DIR/package.xml" ]] || ! grep -q '<name>mocap_localization</name>' "$PKG_DIR/package.xml"; then
    echo "!! $PKG_DIR is not the mocap_localization package." >&2
    echo "   Keep this script inside the package folder (next to package.xml)." >&2
    exit 1
fi

mkdir -p "$SRC"
if [[ "$(realpath "$PKG_DIR")" != "$(realpath -m "$SRC/mocap_localization")" ]]; then
    echo ">> Copying $PKG_DIR -> $SRC/mocap_localization"
    rm -rf "$SRC/mocap_localization"
    cp -r "$PKG_DIR" "$SRC/mocap_localization"
fi

echo ">> System deps (netbase fixes the vrpn getprotobyname() failure)"
apt-get update && apt-get install -y ros-foxy-vrpn-mocap netbase

echo ">> Building"
# ROS setup scripts read unset variables, so relax -u while sourcing
set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
set -u
cd "$WS"
colcon build --packages-select mocap_localization --symlink-install

echo ">> Done. Source it:  source $WS/install/setup.bash"
