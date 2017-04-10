FROM ioft/armhf-ubuntu:trusty
MAINTAINER Adam Ierymenko <adam.ierymenko@zerotier.com>

ENV QEMU_EXECVE 1
COPY qemu/cross-build-end qemu/cross-build-start qemu/qemu-arm-static qemu/sh-shim /usr/bin/
RUN [ "cross-build-start" ]

RUN apt-get update -y
RUN apt-get upgrade -y

RUN apt-get install -y build-essential debhelper libhttp-parser-dev liblz4-dev libnatpmp-dev dh-systemd ruby-ronn g++ make devscripts clang-3.6

RUN ln -sf /usr/bin/clang++-3.6 /usr/bin/clang++
RUN ln -sf /usr/bin/clang-3.6 /usr/bin/clang

RUN dpkg --purge libhttp-parser-dev

RUN update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

RUN sh -c 'echo "deb http:/catkin_test_results build/test_results/packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install ros-indigo-de
RUN sh -c 'echo "deb http:/catkinsktop -y
RUN apt-get install ros-indigo-navigation -y
RUN apt-get install ros-indigo-rgbd-launch -y
RUN apt-get install ros-indigo-depthimage-to-laserscan -y
RUN apt-get install ros-indigo-tf2-sensor-msgs -y
RUN rosdep init
RUN rosdep update
RUN apt-get install libsdl-image1.2-dev -y
RUN apt-get install socat -y
RUN apt-get install ccache -y
RUN apt-get install python-gevent=1.0-1ubuntu1 -y
RUN apt-get install libunwind8-dev -y
RUN apt-get install expect-dev -y

ADD ./ /mfp_workspace/src

RUN sh -c 'chmod +x ./mfp_workspace/src/srsbot_chuck/scripts/build_workspace.sh'
RUN sh -c './mfp_workspace/src/srsbot_chuck/scripts/build_workspace.sh'

RUN [ "cross-build-end" ]
