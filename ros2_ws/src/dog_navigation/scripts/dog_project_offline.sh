#!/usr/bin/env bash
# Orchestrates existing ROS2 packages only; no custom grid conversion algorithm.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/robot/dog_navigation/ws/install/setup.bash
set -u
bag="${1:?Pass existing rosbag directory}"
out="${2:?Pass a NEW output directory}"
test -f "$bag/metadata.yaml"
if test -e "$out"; then printf 'Refusing to overwrite %s\n' "$out" >&2; exit 2; fi
mkdir -p "$out"
# Private localhost DDS domain: replay does not enter the live robot ROS graph.
export ROS_DOMAIN_ID=47 ROS_LOCALHOST_ONLY=1
pids=()
cleanup() {
  for pid in "${pids[@]}"; do
    python3 - "$pid" <<'PY'
import os,signal,sys,time
pid=int(sys.argv[1])
for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGKILL):
    try: os.killpg(pid,sig)
    except ProcessLookupError: break
    for _ in range(20):
        try: os.killpg(pid,0)
        except ProcessLookupError: break
        time.sleep(.1)
    else: continue
    break
PY
  done
}
trap cleanup EXIT
setsid ros2 launch dog_navigation dog_grid_projection.launch.py use_sim_time:=true > "$out/octomap.log" 2>&1 &
pids+=("$!")
setsid ros2 launch dog_navigation dog_mapping.launch.py use_sim_time:=true "map_path:=$out/map.pcd" > "$out/fastlio.log" 2>&1 &
pids+=("$!")
sleep 3
timeout 15 ros2 param get /dog_grid/dog_octomap frame_id > "$out/readiness.txt"
timeout 15 ros2 param get /dog_fastlio common.lid_topic >> "$out/readiness.txt"
ros2 param dump /dog_grid/dog_octomap > "$out/octomap_parameters.yaml"
python3 - "$bag" > "$out/replay.log" 2>&1 <<'PY'
import sys,rosbag2_py
from rclpy.qos import QoSProfile
o=rosbag2_py.PlayOptions()
o.node_prefix='dog_grid_replay_'
o.topics_to_filter=['/dog_lidar/custom','/dog_lidar/imu']
o.clock_publish_frequency=100.0
o.disable_keyboard_controls=True
o.delay=1.0
o.topic_qos_profile_overrides={name:QoSProfile(depth=512) for name in o.topics_to_filter}
rosbag2_py.Player().play(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='sqlite3'),o)
PY
sleep 2
timeout -s INT -k 5s 25s ros2 service call /dog_mapping/save_map std_srvs/srv/Trigger '{}' > "$out/pcd_save.log" 2>&1
test -s "$out/map.pcd"
# PGM unknown gray=205 => occupancy 50/255. free=0.196 keeps it unknown on reload.
# Humble's default free=0.25 would read gray 205 as free space.
timeout -s INT -k 5s 25s ros2 run nav2_map_server map_saver_cli -f "$out/map" --fmt pgm --free 0.196 --occ 0.65 --ros-args \
  -r __node:=dog_map_saver -r map:=/dog_grid/map \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0 > "$out/map_saver.log" 2>&1
test -s "$out/map.pgm"
test -s "$out/map.yaml"
sha256sum "$bag/metadata.yaml" "$out/map.pcd" "$out/map.pgm" "$out/map.yaml" > "$out/hashes.txt"
printf 'SAVED %s/map.yaml\n' "$out"
