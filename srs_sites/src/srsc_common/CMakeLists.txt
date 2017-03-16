#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

project(srsc_common)

####################################################################################################
# Project

project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

####################################################################################################
# Installation

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)