source /opt/ros/indigo/setup.bash
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j8 -l8'
export ROS_LANG_DISABLE=genlisp:roslisp

pushd mfp_workspace

wstool init src

# compile and run unit tests
time catkin_make -j4 run_tests

# generate unified test results
time catkin_test_results -j4 build/test_results

popd