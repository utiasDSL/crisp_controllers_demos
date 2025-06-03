ARG ROS_DISTRO=humble
 
FROM osrf/ros:${ROS_DISTRO}-desktop AS base

ENV ROS_DISTRO=${ROS_DISTRO}
ARG MUJOCO_VERSION=3.2.6

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# HACK: Temporary issue with keys https://discourse.ros.org/t/ros-signing-key-migration-guide/43937/27 ===
RUN rm /etc/apt/sources.list.d/ros2-latest.list \
  && rm /usr/share/keyrings/ros2-latest-archive-keyring.gpg

RUN apt-get update \
  && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros2-apt-source.deb \
    && rm -f /tmp/ros2-apt-source.deb
# ========================================================================================================

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    vim \
    build-essential \
    cmake \
    wget \
    git \
    unzip \
    pip \
    python3-venv \
    python3-ament-package \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-common-extensions \
    cmake \
    libpoco-dev \
    libeigen3-dev \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    # ros-$ROS_DISTRO-rmw-zenoh-cpp \
    dpkg


# Symlink python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

USER $USERNAME

RUN mkdir -p /home/ros/ros2_ws

# === INSTALL MUJOCO ===

WORKDIR /home/ros

ENV MUJOCO_VERSION=${MUJOCO_VERSION}

RUN sudo apt update && sudo apt-get install -y libglfw3-dev wget \ 
    && wget https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz \
    && tar -xzf mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz -C "/home/ros"

WORKDIR /home/ros/ros2_ws
WORKDIR /home/ros/ros2_ws

# NOTE: it would be better to do separate builds for each repo but for testing this is enough
# NOTE: There are still some bugs in the latest versions, for now using this commit
# === FRANKA ROS2 ===
RUN git clone https://github.com/frankaemika/franka_ros2.git src/franka_ros2 --depth 100 \
    && cd src/franka_ros2 \
    && git checkout 39df9c11804ec6db7bb6d80972ce0c68a6918296 \
    && cd /home/ros/ros2_ws \
    && source /opt/ros/humble/setup.bash \
    && sudo apt-get update \
    && vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && touch src/franka_ros2/COLCON_IGNORE src/libfranka/COLCON_IGNORE src/franka_description/COLCON_IGNORE


# === KINOVA ===
RUN git clone https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    # We dont need this for now
    # && vcs import src < src/ros2_kortex/ros2_kortex.${ROS_DISTRO}.repos --recursive --skip-existing \
    # && rosdep install --ignore-src --from-paths src -y -r \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && touch src/ros2_kortex/COLCON_IGNORE

# === IIWA (lbr_fri_ros2_stack would also be an option) ===
RUN git clone https://github.com/ICube-Robotics/iiwa_ros2.git src/iiwa_ros2 \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep install --ignore-src --from-paths . -y -r \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install \
    && touch src/iiwa_ros2/COLCON_IGNORE


# === MOUNT THE REPO then build and remove ===

FROM base AS overlay

COPY . src/crisp_controllers_demos

RUN source /opt/ros/humble/setup.bash \
    && source install/setup.bash \
    && sudo apt update \
    && rosdep update \
    && rosdep install -q --from-paths src --ignore-src -y \ 
    && colcon build --symlink-install --symlink-install \ 
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# USER root
# RUN rm -rf src/crisp_controllers_demos  # Remove it to be able to mount it 
# USER $USERNAME
