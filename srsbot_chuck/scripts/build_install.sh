source /opt/ros/indigo/setup.bash
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j4 -l4'
export ROS_LANG_DISABLE=genlisp:roslisp

if [ ! -f /mfp_workspace/src/CMakeLists.txt ]; then
    pushd /mfp_workspace/src
    catkin_init_workspace
    popd
fi

pushd /mfp_workspace

# compile and run unit tests
time catkin_make run_tests

# generate unified test results
time catkin_test_results build/test_results

popd
