set -euo pipefail

cd /mfp_workspace

# remove any older builds
#rm -rf build/ devel/ install/

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/run_unit_tests.sh"

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/unit_test_results.sh"

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/build_install.sh"

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/deploy_artifact.sh"
