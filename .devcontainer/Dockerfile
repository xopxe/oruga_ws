ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

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
    echo "Defaults env_keep = \"http_proxy ftp_proxy\"" >> /etc/sudoers.d/$USERNAME && \
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
    tilix \
    gsettings-desktop-schemas \
    #ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-dynamixel-hardware \
    #ros-${ROS_DISTRO}-gz-ros2-control \
    #ros-${ROS_DISTRO}-gz-sensors-vendor \
    ros-${ROS_DISTRO}-hls-lfcd-lds-driver \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rtabmap-ros \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-v4l2-camera \
    && sudo rm -rf /var/lib/apt/lists/*

# Fix tilix instalation
RUN sudo ln -s /etc/profile.d/vte-2.91.sh /etc/profile.d/vte.sh \
    && sudo echo -e "if [ \$TILIX_ID ] || [ \$VTE_VERSION ]; then\n source /etc/profile.d/vte.sh\nfi" >> ~/.bashrc

WORKDIR /oruga_ws

ENV ROS_DOMAIN_ID=1

RUN echo "[[ -f /oruga_ws/install/setup.sh ]] && source /oruga_ws/install/setup.sh" >> ~/.bashrc

# Zenoh
RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    && sudo rm -rf /var/lib/apt/lists/*
RUN echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc \
    && echo "export ZENOH_ROUTER_CONFIG_URI=/francesca_ws/zenoh_router_config.json5" >> ~/.bashrc \
    && echo "export ZENOH_SESSION_CONFIG_URI=/francesca_ws/zenoh_session_config.json5" >> ~/.bashrc

CMD ["/bin/bash"]
