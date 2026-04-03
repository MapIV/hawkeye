FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-tf2-eigen \
    ros-noetic-tf2-geometry-msgs \
    libyaml-cpp-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
