#!/usr/bin/env bash
set -e
BASE=/home/robot/dog_vision_custom_20261026
source /opt/ros/humble/setup.bash
exec "$BASE/venv/bin/python" "$BASE/scripts/dog_custom_yolo.py" --model "$BASE/models/dog_fire_chemical_human_yolov8n.onnx" --directory "$BASE/evidence" --port 8082
