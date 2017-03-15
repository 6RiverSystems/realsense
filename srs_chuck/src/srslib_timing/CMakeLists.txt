#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_timing)

set(${PROJECT_NAME}_MSGS
	TimingData.msg
    TimingDataSample.msg
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_LIBRARY ${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################
# Messages & Services

add_message_files(
    FILES
    ${${PROJECT_NAME}_MSGS}
)

generate_messages(
    DEPENDENCIES
		std_msgs
)

####################################################################################################
# Target

catkin_package(
	INCLUDE_DIRS include
	DEPENDS
		roscpp
  		rospy
  		message_runtime
  		std_msgs
)

####################################################################################################
# Installation

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
