#!/usr/bin/env bash
# Install dependencies and build the workspace.
# Usage (from repo root):  ./scripts/setup.sh [ros_distro]     e.g. ./scripts/setup.sh humble
set -euo pipefail

ROS_DISTRO_NAME="${1:-${ROS_DISTRO:-humble}}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
  echo "ROS 2 '${ROS_DISTRO_NAME}' not found in /opt/ros. Install it first (see README)." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

cd "$REPO_ROOT"
sudo apt-get update
sudo apt-get install -y xterm python3-colcon-common-extensions python3-rosdep python3-numpy python3-yaml python3-matplotlib
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then sudo rosdep init; fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
echo
echo "Build finished. Now run:  source install/setup.bash"
