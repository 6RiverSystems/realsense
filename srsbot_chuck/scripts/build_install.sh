set -euo pipefail

# setup environment
source /mfp_workspace/src/srsbot_chuck/scripts/build_env.sh

time catkin_make -DCMAKE_BUILD_TYPE=Release install

