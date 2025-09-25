#!/bin/bash

echo "Starting oruga_start.sh"

source /oruga_ws/install/setup.sh
exec ros2 launch bringup oruga.launch.py "$@"
