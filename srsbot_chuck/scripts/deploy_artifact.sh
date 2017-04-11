set -euo pipefail
IFS=$'\n\t'

PACKAGE_FILE=mfp_chuck

VERSION_FILE=./install/share/srsbot_chuck/package.xml

if [ ! -f $VERSION_FILE ]; then
	echo "Version file ($VERSION_FILE) does not exist" >&2
	exit 100
fi

OUTPUT_DIR=`pwd`/artifacts
VERSION=`xmllint --xpath 'string(//package/version)' $VERSION_FILE`
ARTIFACT=$PACKAGE_FILE-linux-armhf
ARTIFACT_SOURCE_PATH="./install"
ARTIFACT_NAME="${ARTIFACT}-${VERSION}.tar.gz"
ARTIFACT_PATH="${OUTPUT_DIR}/${ARTIFACT_NAME}"

mkdir -p "${OUTPUT_DIR}"

# Create artifact
tar -cvf $ARTIFACT_PATH -C "${ARTIFACT_SOURCE_PATH}" .

if [[ -z "${ARTIFACTORY_API_KEY:-}" ]]; then
	echo "ARTIFACTORY_API_KEY environment variable not set" >&2
	exit 100
fi

# Upload the zipped ros packages to Artifactory
echo 'deploying artifact to artifactory'
curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_API_KEY}" \
	-T "${ARTIFACT_PATH}" \
 	"https://sixriver.jfrog.io/sixriver/binaries/mfp_chuck/${ARTIFACT_NAME}"
