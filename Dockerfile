FROM ioft/armhf-ubuntu:trusty
MAINTAINER Dan Winkler <dan@6river.com>

ADD qemu/qemu-arm-static /usr/bin/

RUN apt-get update -y
RUN apt-get upgrade -y

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update

# Install development packages
RUN apt-get install \
    build-essential \
    debhelper \
    libhttp-parser-dev \
    liblz4-dev \
    libnatpmp-dev \
    dh-systemd \
    ruby-ronn \
    ccache \
    g++ \
    make \
    devscripts \
    libsdl-image1.2-dev \
    python-gevent=1.0-1ubuntu1 \
    libunwind8-dev \
    expect-dev \
    python-wstool \
    python-rosdep \
    ninja-build \
    -y

# Install ROS packages
RUN apt-get install \
    ros-indigo-ros-base \
    ros-indigo-urdf \
    ros-indigo-xacro \
    ros-indigo-navigation \
    ros-indigo-rgbd-launch \
    ros-indigo-filters \
    ros-indigo-depthimage-to-laserscan \
    ros-indigo-tf2-sensor-msgs \
    ros-indigo-diagnostic-updater \
    ros-indigo-camera-info-manager \
    ros-indigo-rviz \
    netpbm \
    -y

ENV CCACHE_DIR /.ccache