
# Oruga

This the ROS2 system for the Oruga robot. The robot itself a pico-ros-based [firmware](https://github.com/xopxe/pico-oruga-platformio-espidf) on its microcontoller.

This system provides:

* Docker image.

* ROS2 Jazzy and newer.

* URDF model for the robot.

![Oruga robot](https://github.com/xopxe/pico-oruga-platformio-espidf/blob/main/docs/oruga.jpg?raw=true)


## Installation

### Using Docker

If you want to use the provided Docker image, you can start it directly from VSCode. You can also build it manually:

```sh
docker image build --rm -t oruga_ws:jazzy .devcontainer/
docker run -it --user ubuntu -v $PWD:/oruga_ws oruga_ws:jazzy \
  /oruga_ws/.devcontainer/postCreateCommand.sh
```

Then you can run from there:

```sh
docker run -it --privileged --user ubuntu --network=host --ipc=host \
  -v $PWD:/oruga_ws -v /dev:/dev --env=DISPLAY oruga_ws:jazzy
```

You can also start the Docker without lanching the robot, start a console and work with ROS 2 as usual:

```sh
docker run -it --privileged --user ubuntu --network=host --ipc=host \
  -v $PWD:/oruga_ws -v /dev:/dev --env=DISPLAY oruga_ws:jazzy /bin/bash

# inside the docker terminal:
colcon build --symlink-install
ros2 launch bringup oruga.launch.py
```

### Local installation

A ROS 2 desktop  install should have most of the needed packages already.

For more information, check the `.devcontainer/Dockerfile` file to see what packages you might need. 

To run the robot call:

```sh
ros2 launch bringup oruga.launch.py
```

### Aditional services

[!TODO]
The host must also be running a zenoh router

[!TODO]
The system must also start a [sync_time](https://github.com/xopxe/ros2_sync_time_service_ws) service

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
