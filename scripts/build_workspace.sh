#!/usr/bin/env bash
source /opt/ros/indigo/setup.bash
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS=-j4

cd /catkin_ws/src && catkin_init_workspace

catkin_make -j4 run_tests
catkin_test_results -j4 build/test_results
catkin_make -j4 install