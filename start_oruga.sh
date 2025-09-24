#!/bin/bash

echo "Starting sync_time_start.sh"

cd /oruga_ws

#source /opt/ros/jazzy/setup.bash

source install/setup.sh

exec ros2 launch bringup oruga.launch.py "$@"
