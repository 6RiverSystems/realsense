set -euo pipefail

export ROS_BUILD_IMAGE=6river/rosbuild-armhf

docker pull $ROS_BUILD_IMAGE

docker run --rm -v $PWD:/mfp_workspace/src -v $PWD/artifacts:/mfp_workspace/artifacts -v $PWD/.ccache:/ccache -e CCACHE_DIR=/ccache --privileged -it $ROS_BUILD_IMAGE /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build.sh"
