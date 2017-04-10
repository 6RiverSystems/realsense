source /opt/ros/indigo/setup.bash
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j8 -l8'
export ROS_LANG_DISABLE=genlisp:roslisp

cd /mfp_workspace

# compile and run unit tests
time catkin_make -j4 install

# generate unified test results
time catkin_test_results -j4 build/test_results
