FROM ros:noetic

# Install ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-tf2-ros \
    ros-noetic-geometry2 \
    ros-noetic-rosbridge-suite \
    ros-noetic-ros-tutorials \
    ros-noetic-rosbag \
    ros-noetic-rosbag-storage \
    ros-noetic-rosbash \
    ros-noetic-rosboost-cfg \
    ros-noetic-rosbridge-server \
    ros-noetic-rosbuild \
    ros-noetic-rosclean \
    ros-noetic-rosconsole \

    # Non-ROS tools
    build-essential \
    cmake \
    vim \
    tree \
    unzip \
    tmux \
    curl \
    python3 \
    python3-pip \
    git \
    valgrind \
    doxygen \
    python3-vcstool

RUN sudo pip3 install -U \
    rosdep \
    rosinstall_generator \
    catkin_tools

# Symbolic link for Python 1
RUN ln -s /usr/bin/python3 /usr/bin/python

# Make LX16A servo library
RUN cd /opt && \
    git clone https://github.com/rjk2147/LX16A.git && \
    cd LX16A && \
    gcc -g -o lx16a.so -shared -fPIC lx16a.c

WORKDIR /root/dev/robot