#!/bin/bash

cd /oruga_ws
git submodule update --init --recursive

sudo -E rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /oruga_ws/install/setup.sh

