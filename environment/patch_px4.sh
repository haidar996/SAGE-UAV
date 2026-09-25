#!/bin/bash
# Install the repository's Gazebo worlds and the drone model (extra nadir camera link) into a PX4-Autopilot checkout.
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PX4="${1:-$HOME/PX4-Autopilot}"
G="$PX4/Tools/simulation/gz"
cp "$REPO"/worlds/*.sdf "$G/worlds/"
[ -f "$G/models/x500_mono_cam/model.sdf.orig" ] || cp "$G/models/x500_mono_cam/model.sdf" "$G/models/x500_mono_cam/model.sdf.orig"
cp "$REPO/models/x500_mono_cam/model.sdf" "$G/models/x500_mono_cam/model.sdf"
echo "worlds and drone model installed into $G (original model saved as model.sdf.orig)"
