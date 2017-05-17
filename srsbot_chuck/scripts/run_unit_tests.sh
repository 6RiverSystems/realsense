set -euo pipefail

# setup environment
source src/srsbot_chuck/scripts/build_env.sh

# compile and run unit tests
time catkin_make -DCMAKE_BUILD_TYPE=Release

. devel/setup.bash

time catkin_make -DCMAKE_BUILD_TYPE=Release run_tests
