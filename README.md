
# Oruga

This the ROS2 system for the Oruga robot. The robot itself a pico-ros-based [firmware](https://github.com/xopxe/pico-oruga-platformio-espidf) on its microcontoller.

This system provides:

* Docker image.

* ROS2 Jazzy and newer.

* URDF model for the robot.

![Oruga robot](https://github.com/xopxe/pico-oruga-platformio-espidf/blob/main/docs/oruga.jpg?raw=true)

## Using Docker

If you want to use the provided Docker image, you can start it directly from VSCode. You can also build it manually:

```sh
# Create the image
docker image build --rm -t oruga_ws:jazzy .devcontainer/

# Initialize the container
docker run -it --user ubuntu -v $PWD:/oruga_ws oruga_ws:jazzy \
  /oruga_ws/.devcontainer/postCreateCommand.sh
```

Then you can start the robot:

```sh
docker run -it --privileged --user ubuntu --network=host --ipc=host \
  -v $PWD:/oruga_ws -v /dev:/dev --env=DISPLAY \
  -e ROS_DOMAIN_ID=100 \
  --name oruga_robot --rm \
  oruga_ws:jazzy
```

You can also start the Docker without launching the robot, start a console and work with ROS 2 as usual:

```sh
# To start the container
docker run -it --privileged --user ubuntu --network=host --ipc=host \
  -v $PWD:/oruga_ws -v /dev:/dev --env=DISPLAY \
  --name oruga_robot --rm \
  oruga_ws:jazzy \
  /bin/bash

# inside the docker terminal:
colcon build --symlink-install
ros2 launch bringup oruga.launch.py
```

You can connect to a running docker to run additional terminals:

```sh
 docker container exec -it oruga_robot /usr/bin/bash
```

> [!TIP]
> Instead of running /bin/bash from the docker you can run tilix, a tiling graphical console.

## Local installation

A ROS 2 desktop  install should have most of the needed packages already.

For more information, check the `.devcontainer/Dockerfile` file to see what packages you might need. This repo pulls in the `sync_time` dependency as a git submodule, so fetch it too when cloning it:

```sh
git clone --recurse-submodules https://github.com/xopxe/oruga_ws.git
```

...or if you already cloned it:

```sh
git submodule update --init --recursive
```

To run the robot call:

```sh
export ROSDOMAIN_ID=100
sudo -E rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.sh
ros2 launch bringup oruga.launch.py
```

You might have to delete the `build` folder if you have compiled from the Docker before.

## Run the docker as `systemd` service

Edit `util/oruga_ws.service` file, set path to this directory in the ExecStart line (just after the `-v`). Then:

```sh
sudo cp -v util/oruga_ws.service /etc/systemd/system
sudo systemctl enable oruga_ws.service
sudo systemctl start oruga_ws.service
```

Verify it is working:

```sh
sudo systemctl status oruga_ws.service
sudo journalctl -f -u oruga_ws.service
```

To uninstall:

```sh
sudo systemctl stop rmw_zenoh_router.service
sudo systemctl disable rmw_zenoh_router.service
sudo systemctl daemon-reload
```

## Aditional services

[!TODO]
The host must also be running a zenoh router

## Controlling the robot

The robot listens for the following topics:

* `/cmd_vel`

The robot emits multiple topics, do a `ros2 topic list` to explore them.

For example, to drive the robot in circles, do:

```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: 'auto', twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"
```

To control the robot with the keyboard (in a separate console), run:

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2025

## License

Apache 2.0
