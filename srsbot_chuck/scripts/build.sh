cd ~/mfp_workspace/src

if [[ -z "$1" || -z "$2" ]]; then
	echo "Usage build.sh <ARCHITECTURE> <ROS_DISTRO>" >&2
	exit 101
fi

set -euo pipefail

export ARCH=$1
export ROS_DISTRO=$2

export ROS_BUILD_IMAGE=6river/rosbuild-$ROS_DISTRO-$ARCH

sudo docker run --rm -v /home/dan/mfp_workspace/$ROS_DISTRO-$ARCH:/mfp_workspace -v $PWD:/mfp_workspace/src -e CCACHE_DIR=/mfp_workspace/ccache -it $ROS_BUILD_IMAGE /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_all.sh"

# Correct permissions from docker build
sudo chown dan:dan -R ~/mfp_workspace