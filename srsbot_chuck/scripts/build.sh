cd ~/mfp_workspace/src

if [[ -z "$1" || -z "$2" ]]; then
	echo "Usage build.sh <ARCHITECTURE> <ROS_DISTRO> <ARTIFACTORY_API_KEY> - <ARTIFACTORY_API_KEY> is optional" >&2
	exit 101
fi

if [[ -z "3" ]]; then
    export ARTIFACTORY_API_KEY
else
    export ARTIFACTORY_API_KEY=$3
fi

set -euo pipefail

export ARCHITECTURE=$1
export ROS_DISTRO=$2

export ROS_BUILD_IMAGE=6river/rosbuild-$ROS_DISTRO-$ARCHITECTURE
export ROS_WORKSPACE=~/mfp_workspace

cd "$ROS_WORKSPACE"

pushd "$ROS_WORKSPACE/src"

# Update all submodules
git submodule init
git submodule update --recursive

popd

sudo docker run \
    --rm \
    -v $PWD/$ROS_DISTRO-$ARCHITECTURE:/mfp_workspace \
    -v $PWD/src:/mfp_workspace/src \
    -e ROS_DISTRO=$ROS_DISTRO \
    -e ARCHITECTURE=$ARCHITECTURE \
    -e ARTIFACTORY_API_KEY=$ARTIFACTORY_API_KEY \
    -e CCACHE_DIR=/mfp_workspace/ccache \
    -it $ROS_BUILD_IMAGE \
    /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_all.sh"

# Correct permissions from docker build
sudo chown $USER:$USER -R "$ROS_WORKSPACE"