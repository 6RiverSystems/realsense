FROM ioft/armhf-ubuntu:trusty
MAINTAINER Adam Ierymenko <adam.ierymenko@zerotier.com>

ADD qemu-arm-static /usr/bin/

RUN apt-get update -y
RUN apt-get upgrade -y

RUN apt-get install -y build-essential debhelper libhttp-parser-dev liblz4-dev libnatpmp-dev dh-systemd ruby-ronn g++ make devscripts clang-3.6

RUN ln -sf /usr/bin/clang++-3.6 /usr/bin/clang++
RUN ln -sf /usr/bin/clang-3.6 /usr/bin/clang

RUN dpkg --purge libhttp-parser-dev

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update
RUN apt-get install ros-indigo-desktop -y
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

RUN /bin/bash -c "source /opt/ros/indigo/setup.bash"
RUN /bin/bash -c "export PATH=/usr/lib/ccache:$PATH"
RUN /bin/bash -c "export ROS_PARALLEL_JOBS=-j4"

ADD ./ /catkin_ws/src/project

RUN /bin/bash -c "cd /catkin_ws/src"
RUN /bin/bash -c "catkin_init_workspace"
RUN /bin/bash -c "cd /catkin_ws"
#RUN /bin/bash -c "catkin_make run_tests -j4"
#RUN /bin/bash -c "catkin_test_results ~/catkin_ws/build/test_results"

#RUN /bin/bash -c "catkin_make install -j4"
