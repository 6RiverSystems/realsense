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