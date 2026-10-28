#!/usr/bin/env bash
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec ros2 run rviz2 rviz2 -d "$AUTORACE_ROOT/src/limo_autorace_sim/rviz/autorace.rviz" \
  --ros-args -p use_sim_time:=true
