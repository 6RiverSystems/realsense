
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

  unbuffer roslaunch "srsc_$ROS_MAP" map.launch 2>&1 | tee $logFile
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

function showChuck() {
  source /opt/ros/indigo/setup.bash

  killall electron
  killall electron

  export DISPLAY=:0

  rviz -d ~/ros/srs_sites/src/srsc_6rhq_rviz/rviz/config.rviz
}

function stopChuck() {
  echo "Stopping mfp-ros service"
  sudo service mfp-ros stop
  
  echo "Stopping mfp-bridge service"
  sudo service mfp-bridge stop
}

function startChuck() {
  echo "Starting mfp-ros service"
  sudo service mfp-ros start

  echo "Stopping mfp-bridge service"
  sudo service mfp-bridge start
}

function restartChuck() {
  stopChuck

  echo "Waiting for services to completely stop"
  sleep 10s

  startChuck
}

logChuck() {
  getChuckDirectory &&

  tail -n 1000 -f "$logFile" -f ~/mfp_bridge/log/bridge.log
}

recordChuck() {
  getChuckDirectory

  if [ -z "$1" ]; then
      echo "No argument supplied"
  else
    echo "Recording realsense bag: $1.bag"
    rosbag record -O "$1.bag" /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_raw /camera/infrared1/camera_info /camera/infrared1/image_raw /camera/infrared2/camera_info /camera/infrared2/image_raw

    echo "Compressing bag: $1.bag => $1.tar.gz"
    tar -czvf "$1.tar.gz" "$1.bag"

    echo "Removing uncompressed bag $1"
    rm "$1.bag"
  fi
}

alias chucklog=logChuck
alias chuckupdate=updateChuck
alias chuckclean=cleanChuck
alias chuckbuild=buildChuck
alias chuckrun=runChuck
alias chuckstart=startChuck
alias chuckstop=stopChuck
alias chuckrestart=restartChuck
alias chuckshow=showChuck
alias chuckrecord=recordChuck
