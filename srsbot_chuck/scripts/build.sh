set -euo pipefail

/bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_unit_tests.sh"

/bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/build_install.sh"

/bin/bash -C "/mfp_workspace/src/srsbot_chuck/scripts/deploy_artifact.sh"
