#!/bin/bash
function getChuckDirectory() {
  BLUE='\033[0;34m'
  YELLOW='\033[0;33m'
  NC='\033[0m' # No Color

  baseDirectory=~/ros
  if [ "$ENV" != "ros" ]; then
    baseDirectory="/home/rivs/projects/$ENV/ros"
  fi

  logDirectory="$baseDirectory/log/ros.log"

  echo -e "${BLUE}*************************************************************"
  echo -e "${BLUE}*** ${YELLOW}ENV: $ENV"
  echo -e "${BLUE}*** ${YELLOW}ROS directory: $baseDirectory"
  echo -e "${BLUE}*** ${YELLOW}Log file: $logDirectory"
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

  roslaunch srsc_6rhq_norviz map.launch 2>&1 | tee "$baseDirectory/log/ros.log"
}

function cleanChuck() {
  getChuckDirectory &&

  pushd "$baseDirectory/srs_chuck" &&
  rm -rf build/ devel/ install &&
  popd &&

  pushd "$baseDirectory/srs_sites" &&
  rm -rf build/ devel/ install &&
  popd
}

logChuck() {
  tail -f "$baseDirectory/log/ros.log" ~/mfp_bridge/log/bridge.log
}

alias chucklog=logChuck
alias chuckupdate=updateChuck
alias chuckclean=cleanChuck
alias chuckbuild=buildChuck
alias chuckrun=runChuck
