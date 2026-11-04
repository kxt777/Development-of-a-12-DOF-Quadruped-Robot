#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/dog_env.sh"
cd "$DOG_FINAL_ROOT/ros2_ws"
colcon build --symlink-install --parallel-workers 2 \
  --packages-select dm_imu dog_ik_walk dog_navigation dog_description dog_perception dog_bringup
printf '\nBuild finished. Run: cd %s && ./dog help\n' "$DOG_FINAL_ROOT"
