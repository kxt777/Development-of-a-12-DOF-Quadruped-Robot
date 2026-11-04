#!/usr/bin/env bash
# Optional rebuild of the captured patched dependencies; no service changes.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/dog_env.sh"
mkdir -p "$DOG_FINAL_ROOT/vendor_ws"
cd "$DOG_FINAL_ROOT/vendor_ws"
colcon build --base-paths "$DOG_FINAL_ROOT/reference/third_party" \
  --build-base build --install-base install --symlink-install --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
printf '\nCaptured drivers built; run ./dog build again from the project root.\n'
