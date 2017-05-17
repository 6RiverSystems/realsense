set -euo pipefail

if [ "$(lsb_release -sc)" = "trusty" ]; then
    ROS_DISTRO=indigo
elif [ "$(lsb_release -sc)" = "xenial" ]; then
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

# generate unified test results
time catkin_test_results build/test_results
