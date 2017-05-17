set -euo pipefail

cd ~/mfp_workspace/src

export ARCH=x86_64
export ROS_DISTRO=indigo

export ROS_BUILD_IMAGE=6river/rosbuild-$ROS_DISTRO-$ARCH

sudo docker run --rm -v /home/dan/mfp_workspace/$ROS_DISTRO-$ARCH:/mfp_workspace -v $PWD:/mfp_workspace/src -e CCACHE_DIR=/mfp_workspace/ccache -it $ROS_BUILD_IMAGE /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_all.sh"

sudo chmod dan:dan -R ~/mfp_workspace/artifacts