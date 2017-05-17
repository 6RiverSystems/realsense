set -euo pipefail

ARCH=$(lsb_release -sc)

if [ "$ARCH" = "trusty" ]; then
    ROS_DISTRO=indigo
elif [ "$ARCH" = "xenial" ]; then
    ROS_DISTRO=kinetic
fi

export TMPDIR=/tmp
export ROS_MASTER_URI=http://localhost:11311
export PATH=/usr/lib/ccache:$PATH
export ROS_PARALLEL_JOBS='-j4 -l4'
export ROS_LANG_DISABLE=genlisp

source /opt/ros/$ROS_DISTRO/setup.bash

cd /mfp_workspace

pushd src

if [ ! -f ./CMakeLists.txt ]; then
    catkin_init_workspace
fi

popd

# compile and run unit tests
time catkin_make -DCMAKE_BUILD_TYPE=Release

. devel/setup.bash

time catkin_make -DCMAKE_BUILD_TYPE=Release run_tests
