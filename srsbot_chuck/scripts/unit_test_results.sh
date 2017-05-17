set -euo pipefail

cd /mfp_workspace

# setup environment
source src/srsbot_chuck/scripts/build_env.sh

# generate unified test results
time catkin_test_results /mfp_workspace/build/test_results
