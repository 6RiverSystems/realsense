#!/bin/bash
function getChuckDirectory() {
  BLUE='\033[0;34m'
  YELLOW='\033[0;33m'
  NC='\033[0m' # No Color

  baseDirectory=~/ros
  if [ "$ENV" != "ros" ]; then
    baseDirectory="/home/rivs/projects/$ENV/ros"
  fi

  logFile="$baseDirectory/log/ros.log"

  echo -e "${BLUE}*************************************************************"
  echo -e "${BLUE}*** ${YELLOW}ENV: $ENV"
  echo -e "${BLUE}*** ${YELLOW}ROS directory: $baseDirectory"
  echo -e "${BLUE}*** ${YELLOW}Log file: $logFile"
  echo -e "${BLUE}*************************************************************"
  echo -e "${NC}"
}

function updateChuck() {
  getChuckDirectory &&

  pushd "$baseDirectory" &&
  git pull --rebase &&
  popd
}

function buildChuck() {
  getChuckDirectory &&

  pushd "$baseDirectory/srs_chuck" &&
  catkin_make &&
  source devel/setup.bash &&
  popd &&
  pushd "$baseDirectory/srs_sites" &&
  catkin_make &&
  source devel/setup.bash &&
  popd
}

function runChuck() {
  getChuckDirectory &&

  pushd "$baseDirectory/srs_sites" &&
  source devel/setup.bash &&
  popd

  echo -e "${BLUE}*************************************************************"
  echo -e "${BLUE}*** Running roslaunch from `pwd`"
  echo -e "${BLUE}*** ${YELLOW}ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
  echo -e "${BLUE}*** ${YELLOW}ROSCONSOLE_CONFIG_FILE: $ROSCONSOLE_CONFIG_FILE"
  echo -e "${BLUE}*************************************************************"
  echo -e "${NC}"

  exec roslaunch "srsc_$ROS_MAP" map.launch 2>&1 | tee $logFile
}

function cleanChuck() {
  getChuckDirectory &&

  source /opt/ros/indigo/setup.bash
  pushd "$baseDirectory/srs_chuck" &&
  rm -rf build/ devel/ install &&
  popd &&

  pushd "$baseDirectory/srs_sites" &&
  rm -rf build/ devel/ install &&
  popd
}

logChuck() {
  getChuckDirectory &&

  tail -n 1000 -f "$logFile" -f ~/mfp_bridge/log/bridge.log
}

alias chucklog=logChuck
alias chuckupdate=updateChuck
alias chuckclean=cleanChuck
alias chuckbuild=buildChuck
alias chuckrun=runChuck
