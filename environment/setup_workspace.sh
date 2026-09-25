#!/bin/bash
# Recreate the ROS 2 workspace layout used by every script in this repository:
#   ~/sage_ws/src/px4_msgs              (PX4 message definitions, v1.16.2)
#   ~/sage_ws/src/sage_px4_interface -> <this repo>/src/sage_px4_interface   (symlink)
# Non-destructive: existing directories are kept. Run from the repository root.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p ~/sage_ws/src
[ -d ~/sage_ws/src/px4_msgs ] || git clone --branch v1.16.2 https://github.com/PX4/px4_msgs.git ~/sage_ws/src/px4_msgs
[ -e ~/sage_ws/src/sage_px4_interface ] || ln -s "$REPO/src/sage_px4_interface" ~/sage_ws/src/sage_px4_interface
source /opt/ros/humble/setup.bash
cd ~/sage_ws && colcon build --packages-select px4_msgs sage_px4_interface
echo "workspace ready: source ~/sage_ws/install/setup.bash"
