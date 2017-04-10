#!/usr/bin/env bash
source /opt/ros/indigo/setup.bash
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS=-j4

mkdir -p ~/mfp_workspace
cd ~/mfp_workspace
wstool init src

// compile and run unit tests
catkin_make -j4 run_tests

// generate unified test results
#catkin_test_results -j4 build/test_results