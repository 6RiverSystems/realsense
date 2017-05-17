cd ~/mfp_workspace/src

if [[ -z "$1" || -z "$2" ]]; then
	echo "Usage build.sh <ARCHITECTURE> <ROS_DISTRO>" >&2
	exit 101
fi

set -euo pipefail

export LINUX_DISTRO=$1
export ROS_DISTRO=$2

export ROS_BUILD_IMAGE=6river/rosbuild-$ROS_DISTRO-$LINUX_DISTRO
export ROS_WORKSPACE=~/mfp_workspace

cd "$ROS_WORKSPACE"

sudo docker run \
    --rm \
    -v $PWD/$ROS_DISTRO-$LINUX_DISTRO:/mfp_workspace \
    -v $PWD/src:/mfp_workspace/src \
    -e ROS_DISTRO=$ROS_DISTRO \
    -e LINUX_DISTRO=$LINUX_DISTRO \
    -e CCACHE_DIR=/mfp_workspace/ccache \
    -it $ROS_BUILD_IMAGE \
    /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_all.sh"

# Correct permissions from docker build
sudo chown $USER:$USER -R "$ROS_WORKSPACE"