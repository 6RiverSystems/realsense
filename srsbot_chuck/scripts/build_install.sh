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

# compile and run unit tests
rm -rf build/ devel/ install/

time catkin_make -DCMAKE_BUILD_TYPE=Release install

