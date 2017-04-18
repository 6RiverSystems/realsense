source /opt/ros/indigo/setup.bash

set -euo pipefail

export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j4 -l4'
export ROS_LANG_DISABLE=genlisp

if [ ! -f src/CMakeLists.txt ]; then
    pushd src
    catkin_init_workspace
    popd
fi

rm -rf build/ devel/ install/

# compile and run unit tests
time catkin_make -DCMAKE_BUILD_TYPE=Release install

# generate unified test results
time catkin_test_results build/test_results
