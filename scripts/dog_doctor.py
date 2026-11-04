#!/usr/bin/env python3
"""Read-only checks; deliberately does not open serial ports or start ROS nodes."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros2_ws/src/dog_bringup'))
from dog_bringup.configuration import read_site, resolve
from dog_bringup.preflight import process_commands, conflicts

site = read_site(ROOT)
print('Release:', site['release'])
print('Root:', ROOT)
errors = []
for key in ['lidar_config', 'camera_config', 'mounting_config', 'map_yaml', 'model']:
    path = resolve(ROOT, site[key])
    print(('OK' if path.is_file() else 'MISSING') + ' ' + key + ': ' + str(path))
    if not path.is_file(): errors.append(key)
if resolve(ROOT, site['model']).is_file():
    digest = hashlib.sha256(resolve(ROOT, site['model']).read_bytes()).hexdigest()
    print('ONNX SHA256:', digest)
    if digest != 'dcd1544872485062ee3bd7619914442992dc4c9ee89db4f55c325791b5f28cb9':
        errors.append('model hash differs')
print('Motor interfaces: monitor-only in dog_system.launch.py.')
mount = json.loads(resolve(ROOT, site['mounting_config']).read_text())
print('Footprint:', mount.get('footprint_mode', 'not specified'))
if Path('/proc').is_dir():
    owners = conflicts({'controller', 'imu', 'lidar', 'mapping', 'vision', 'navigation'}, process_commands())
    print('Existing owners (kept running):')
    for name, pid, command in owners:
        print(f'  {name} PID {pid}: {command[:180]}')
if Path('/opt/ros/humble/setup.bash').is_file():
    check = 'source "$1"; for p in dm_imu dog_ik_walk dog_navigation dog_description dog_perception dog_bringup fast_lio livox_ros_driver2 depthai_ros_driver_v3 nav2_planner; do ros2 pkg prefix "$p" || exit 1; done'
    result = subprocess.run(['bash', '-c', check, 'dog_doctor', str(ROOT/'scripts/dog_env.sh')], text=True)
    if result.returncode: errors.append('ROS package discovery; run ./dog build')
    python = ROOT/'.venv/bin/python'
    if not python.is_file(): python = Path(site['vision_python'])
    if python.is_file():
        result = subprocess.run([str(python), '-c', 'import cv2,numpy,onnxruntime; print("Vision Python OK:",onnxruntime.__version__)'])
        if result.returncode: errors.append('vision Python imports')
    else:
        errors.append('vision Python missing')
else:
    print('ROS checks skipped on this computer; run ./dog doctor on the NUC.')
print('RESULT:', 'OK' if not errors else '; '.join(errors))
raise SystemExit(bool(errors))
