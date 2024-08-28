# Use the official ROS Noetic base image
FROM ros:noetic-ros-core

# Set the working directory
WORKDIR /root

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    vim tmux \
    && rm -rf /var/lib/apt/lists/*

# Install tools to build ros ws
RUN apt-get update && apt-get install -y \
    ros-noetic-eigenpy \
    libjsoncpp-dev \
    python3-wstool \
    python3-pybind11 \
    python3-catkin-tools

# Initialize rosdep
RUN rosdep init && rosdep update

# Create a ROS workspace and copy the package
RUN mkdir -p /root/ros1_ws/src

# Set up the environment
ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=noetic

# Source the ROS setup script
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Copy the package and install dependencies
COPY ./ /root/ros1_ws/src/motoman_ros1
RUN cd /root/ros1_ws/src \
    && wstool init . \
    && wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall \
    && wstool remove moveit_tutorials \
    && wstool update -t .
RUN cd /root/ros1_ws/ \
    && rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
WORKDIR /root/ros1_ws
#RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source the workspace setup script
RUN echo "source /root/ros1_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /root/
USER root

SHELL ["/bin/bash", "-c"]
