#!/usr/bin/env bash
set -e
BASE=/home/robot/dog_vision_custom_20261026
source /opt/ros/humble/setup.bash
export DEPTHAI_AUTOCALIBRATION=OFF
export DEPTHAI_TELEMETRY=0
exec ros2 launch depthai_ros_driver_v3 driver.launch.py name:=dog_oak namespace:=/dog_vision parent_frame:=dog_oak_mount params_file:="$BASE/config/dog_oak.yaml" use_rviz:=false
