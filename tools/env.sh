#!/usr/bin/env bash
# Source in every simulation terminal; use an isolated domain by default.
AUTORACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$AUTORACE_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export GZ_PARTITION="${GZ_PARTITION:-autorace_local}"
