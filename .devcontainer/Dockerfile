FROM osrf/ros:jazzy-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

ENTRYPOINT [ "/bin/bash", "/ros_entrypoint.sh" ]

#RUN sudo apt-get update && sudo apt-get install apt-utils

RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-gz-sensors-vendor \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    && sudo rm  -rf /var/lib/apt/lists/*

WORKDIR /oruga_ws

#RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
#    colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON"

RUN echo "export GZ_VERSION=harmonic" >> ~/.bashrc
RUN echo "source /oruga_ws/install/setup.sh" >> ~/.bashrc

CMD ["/bin/bash"]
