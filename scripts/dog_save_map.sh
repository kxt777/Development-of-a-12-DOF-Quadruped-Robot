#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/dog_env.sh"
run=$(python3 -c 'import json,os; from pathlib import Path; root=Path(os.environ["DOG_FINAL_ROOT"]); s=json.loads((root/"state/mapping_run.json").read_text()); p=Path("/proc",str(s["pid"]),"stat"); assert p.exists() and p.read_text().split(") ",1)[1].split()[19]==s["start_ticks"], "Mapping launch is no longer running"; assert s["mode"] in ("mapping","survey"); print(s["directory"])')
if [[ -e "$run/grid/map.yaml" || -e "$run/grid/map.pgm" ]]; then
    printf 'Grid already saved for this run; existing maps preserved.\n' >&2
    exit 1
fi
ros2 service call /dog_mapping/save_map std_srvs/srv/Trigger '{}'
if [[ ! -s "$run/map.pcd" ]]; then
    printf 'No nonempty PCD was produced: %s/map.pcd\n' "$run" >&2
    exit 1
fi
ros2 run nav2_map_server map_saver_cli -f "$run/grid/map" --fmt pgm --free 0.196 --occ 0.65 \
    --ros-args -r __node:=dog_map_saver -r map:=/dog_grid/map \
    -p map_subscribe_transient_local:=true
printf '\nSaved PCD and occupancy grid in: %s\n' "$run"
