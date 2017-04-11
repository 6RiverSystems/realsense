set -e
docker run -v $PWD:/mfp_workspace/src -v $PWD/.ccache:/ccache -e CCACHE_DIR=/ccache --privileged -it ros-build-debian-armhf /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/run_unit_tests.sh"

docker run -v $PWD:/mfp_workspace/src -v $PWD/.ccache:/ccache -e CCACHE_DIR=/ccache --privileged -it ros-build-debian-armhf /bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_install.sh"

srsbot_chuck/scripts/deploy_artifact.sh
