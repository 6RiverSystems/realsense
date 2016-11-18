#!/bin/bash
function getChuckDirectory() {
  BLUE='\033[0;34m'
  YELLOW='\033[0;33m'
  NC='\033[0m' # No Color

  baseDirectory=~/ros

  logFile="$baseDirectory/log/ros.log"
}

function printInfo() {
  echo -e "${BLUE}*************************************************************"
  echo -e "${BLUE}*** ${YELLOW}ROS directory: $baseDirectory"
  echo -e "${BLUE}*** ${YELLOW}Log file: $logFile"
  echo -e "${BLUE}*************************************************************"
  echo -e "${NC}"
}

function updateChuck() {
  getChuckDirectory &&
  printInfo &&

  pushd "$baseDirectory" &&
  git pull --rebase &&
  popd
}

function buildChuck() {
  source /opt/ros/indigo/setup.bash

  getChuckDirectory &&
  printInfo &&

  pushd "$baseDirectory/srs_chuck" &&
  catkin_make -DCMAKE_BUILD_TYPE=Release -j8 &&
  source devel/setup.bash &&
  popd &&
  pushd "$baseDirectory/srs_sites" &&
  catkin_make -DCMAKE_BUILD_TYPE=Release -j8 &&
  source devel/setup.bash &&

  source "$baseDirectory/srs_sites/devel/setup.bash"

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

  stdbuf -i0 -o0 -e0 chuck run 2>&1 | tee $logFile
}

function cleanChuck() {
  getChuckDirectory &&
  printInfo &&

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

  rviz -d ~/ros/srs_chuck/src/srsbot_chuck/rviz/config.rviz
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

  startChuck
}

logChuck() {
  getChuckDirectory &&

  tail -n 1000 -f "$logFile" -f ~/mfp_bridge/log/bridge.log
}

recordChuck() {
  #rosbag record --split --buffsize=0 --duration=5m /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_raw /camera/infrared1/camera_info /camera/infrared1/image_raw /camera/infrared2/camera_info /camera/infrared2/image_raw
rosbag record -O slam-lidar-rs /scan /internal/sensors/odometry/velocity /tf /camera/depth/image_raw /camera/depth/camera_info /camera/rgb/image_raw /camera/rgb/camera_info
}

recordChuckNav() {
  #rosbag record --split --buffsize=0 --duration=5m /camera/color/camera_info /camera/color/image_raw /camera/depth/camera_info /camera/depth/image_raw /camera/infrared1/camera_info /camera/infrared1/image_raw /camera/infrared2/camera_info /camera/infrared2/image_raw
rosbag record -O nav-data /internal/sensors/lidar/scan/raw /internal/sensors/odometry/velocity/cmd /internal/sensors/odometry/velocity/estimate /internal/sensors/odometry/velocity/pose /internal/sensors/odometry/rpm/raw /internal/sensors/odometry/rpm/cmd /tf
}



alias chucklog=logChuck
alias chuckclean=cleanChuck
alias chuckbuild=buildChuck
alias chuckstart=startChuck
alias chuckstop=stopChuck
alias chuckrestart=restartChuck
alias chuckshow=showChuck
alias chuckrecord=recordChuck
alias chuckrecordnav=recordChuckNav
