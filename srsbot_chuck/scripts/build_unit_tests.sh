source /opt/ros/indigo/setup.bash

set -euo pipefail

export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j4 -l4'
export ROS_LANG_DISABLE=genlisp

pushd src

if [ ! -f ./CMakeLists.txt ]; then
    catkin_init_workspace
fi

# update submodules
git submodule init
git submodule update --recursive

popd

# compile and run unit tests
time catkin_make -DCMAKE_BUILD_TYPE=Release run_tests

# generate unified test results
time catkin_test_results build/test_results
