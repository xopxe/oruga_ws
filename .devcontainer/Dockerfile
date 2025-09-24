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
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-sdformat-urdf \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-xacro \
    && sudo rm -rf /var/lib/apt/lists/*

# Fix tilix instalation
RUN sudo ln -s /etc/profile.d/vte-2.91.sh /etc/profile.d/vte.sh \
    && sudo echo -e "if [ \$TILIX_ID ] || [ \$VTE_VERSION ]; then\n source /etc/profile.d/vte.sh\nfi" >> ~/.bashrc

WORKDIR /oruga_ws

ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV ZENOH_ROUTER_CONFIG_URI=/oruga_ws/zenoh_router_config.json5
ENV ZENOH_SESSION_CONFIG_URI=/oruga_ws/zenoh_session_config.json5

RUN echo "[[ -f /oruga_ws/install/setup.sh ]] && source /oruga_ws/install/setup.sh" >> ~/.bashrc

CMD ["/oruga_ws/start_oruga.sh"]
