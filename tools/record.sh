#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
AUTORACE_BAG="${1:-$AUTORACE_ROOT/bags/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname -- "$AUTORACE_BAG")"
exec ros2 bag record --use-sim-time -o "$AUTORACE_BAG" \
  /clock /cmd_vel /sim/cmd_vel /odom /joint_states /tf /tf_static \
  /camera/color/image_raw /camera/color/camera_info /imu /scan \
  /camera/depth/image_raw /camera/depth/camera_info /evaluation/ground_truth
