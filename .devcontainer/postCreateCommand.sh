#!/bin/bash

cd /oruga_ws
sudo -E rosdep install --from-paths src --ignore-src -y

# provide a default version (useful when running in a login shell)
if [[ -z "${GZ_VERSION}" ]]; then
  export GZ_VERSION=harmonic
fi

colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
source /oruga_ws/install/setup.sh

