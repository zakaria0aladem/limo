#!/usr/bin/env bash
# ============================================================================
# install_mocap_localization.sh
# ----------------------------------------------------------------------------
# TODO(zakaria): the original of this script was not recovered. This is a
# reconstructed best-effort stub based on docs/SETUP.md (Part 3). Verify each
# step against your working container, then delete this TODO banner.
#
# Purpose: drop the mocap_localization package into the mounted workspace and
# build it inside the Foxy container.
# ============================================================================
set -e

WS="${ROS2_WS:-/root/ros2_ws}"
SRC="$WS/src"

echo ">> Ensuring workspace src exists at $SRC"
mkdir -p "$SRC"

# If this repo is cloned somewhere the container can see (e.g. a mounted path),
# symlink or copy the package into the workspace src. Adjust REPO_PKG to point
# at this package's directory.
REPO_PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo ">> Linking $REPO_PKG -> $SRC/mocap_localization"
ln -sfn "$REPO_PKG" "$SRC/mocap_localization"

echo ">> System deps (netbase fixes the vrpn getprotobyname() failure)"
apt-get update && apt-get install -y ros-foxy-vrpn-mocap netbase

echo ">> Building"
cd "$WS"
colcon build --packages-select mocap_localization --symlink-install

echo ">> Done. Source it:  source $WS/install/setup.bash"
