#!/usr/bin/env bash
# Source this file. ROS setup scripts are not compatible with `set -u`.
export DOG_FINAL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
while IFS= read -r setup; do
    if [[ -f "$setup" ]]; then source "$setup"; fi
done < <(python3 -c 'import json,os; from pathlib import Path; print("\n".join(json.loads((Path(os.environ["DOG_FINAL_ROOT"])/"config/site.json").read_text())["dependency_underlays"]))')
if [[ -f "$DOG_FINAL_ROOT/vendor_ws/install/setup.bash" ]]; then
    source "$DOG_FINAL_ROOT/vendor_ws/install/setup.bash"
fi
if [[ -f "$DOG_FINAL_ROOT/ros2_ws/install/setup.bash" ]]; then
    source "$DOG_FINAL_ROOT/ros2_ws/install/setup.bash"
fi
