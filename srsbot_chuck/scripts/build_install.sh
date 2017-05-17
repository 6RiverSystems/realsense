set -euo pipefail

# setup environment
source src/srsbot_chuck/scripts/build_env.sh

# update submodules
git submodule init
git submodule update --recursive

popd

time catkin_make -DCMAKE_BUILD_TYPE=Release install

