#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

if [[ -z "${ARTIFACTORY_API_KEY:-}" ]]; then
	echo "ARTIFACTORY_API_KEY environment variable not set" >&2
	exit 100
fi

readonly PACKAGE_FILE=mfp_chuck

readonly VERSION_FILE=../share/srsbot_chuck/package.xml

readonly OUTPUT_DIR=`pwd`/artifacts
readonly VERSION=`xmllint --xpath 'string(//package/version)' $VERSION_FILE`
readonly ARTIFACT=$PACKAGE_FILE-linux-armhf
readonly ARTIFACT_SOURCE_PATH="./install"
readonly ARTIFACT_NAME="${ARTIFACT}-${VERSION}.tar.gz"
readonly ARTIFACT_PATH="${OUTPUT_DIR}/${ARTIFACT_NAME}"

# create the artifact
echo "creating artifact ${ARTIFACT_PATH} for upload from ${ARTIFACT_SOURCE_PATH}"
mkdir -p "${OUTPUT_DIR}"
tar --create --gzip --file "${ARTIFACT_PATH}" \
	--directory "${ARTIFACT_SOURCE_PATH}" .

# deploy artifact
echo 'deploying artifact to artifactory'
curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_API_KEY}" \
	-T "${ARTIFACT_PATH}" \
	"https://sixriver.jfrog.io/sixriver/binaries/mfp_chuck/${ARTIFACT_NAME}"
