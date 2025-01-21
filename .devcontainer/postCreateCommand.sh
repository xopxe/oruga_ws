cd /oruga_ws
sudo rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
