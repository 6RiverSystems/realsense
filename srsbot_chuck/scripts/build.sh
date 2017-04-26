set -euo pipefail

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/build_unit_tests.sh"

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/build_install.sh"

/bin/bash "/mfp_workspace/src/srsbot_chuck/scripts/deploy_artifact.sh"
