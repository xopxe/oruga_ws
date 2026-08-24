#!/bin/bash

cd /oruga_ws/src
git clone https://github.com/xopxe/sync_time.git

cd /oruga_ws
sudo -E rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /oruga_ws/install/setup.sh

