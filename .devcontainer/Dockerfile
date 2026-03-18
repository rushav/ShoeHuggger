FROM ros:humble

# Create ubuntu user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
RUN sudo usermod --append --groups video $USERNAME

# System packages
RUN sudo apt-get update && sudo apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-slam-toolbox \
    ros-humble-compressed-image-transport \
    ros-humble-cv-bridge \
    ros-humble-usb-cam \
    python3-pip \
    xterm \
    git \
    && sudo rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip3 install --no-cache-dir \
    ultralytics \
    opencv-python \
    numpy \
    pynput

# ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source ~/turtlebot4_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=117" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc

# Rosdep
RUN rosdep update

CMD ["bash"]
