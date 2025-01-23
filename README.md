
# Oruga

This the ROS2 system for the Oruga robot. The robot itself a micro-ros-based [firmware](https://github.com/xopxe/micro_rosso_oruga) on its microcontoller.

This system provides:

* Docker image.

* ROS2 Jazzy and newer.

* URDF model for the robot.

Pending:

* Gazebo integration.

## Installation

### Using Docker

If you want to use the provided Docker image, you can use it directly from VSCode. You can also build it manually:

```sh
docker image build --rm -t oruga_ws:jazzy .devcontainer/
```

Then you can run from there:

```sh
docker run -it --privileged --user ubuntu --network=host --ipc=host \
  -v $PWD:/oruga_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  --env=DISPLAY \
 oruga_ws:jazzy /bin/bash
```

### Local installation

If you want to install locally, check the `.devcontainer/Dockerfile` file to see what packages are needed:

```sh
sudo apt-get install -y \
 ros-${ROS_DISTRO}-joint-state-publisher-gui \
 ros-${ROS_DISTRO}-joy \
 ros-${ROS_DISTRO}-teleop-twist-joy
```

In Docker, the project is placed in the `/oruga_ws` directory. To rebuild the project, call:

```sh
colcon build --cmake-args -DBUILD_TESTING=ON --symlink-install
```

> [!TIP]
> If the build fails, try removing the old `install` and `build` directories.

## Running

To run the robot call:

```sh
ros2 launch bringup oruga.launch.py
```

This will not open any window by default, but you can make it open an RViz visualization by passing a `gui:=true` parameter.

## Controlling the robot

The robot listens for the following topics:

* `/cmd_vel`

* `/joy`

The robot emits multiple topics, do a `ros2 topic list` to explore them.

For example, to drive the robot in circles, do:

```sh
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped "{header: 'auto', twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}}"

```

As another example, you can control the robot with a joystick running the provided launch file (in a separate console):

```sh
ros2 launch bringup joystick.launch.py
```

## Authors and acknowledgment

<jvisca@fing.edu.uy> - [Grupo MINA](https://www.fing.edu.uy/inco/grupos/mina/), Facultad de Ingeniería - Udelar, 2025

## License

Apache 2.0
