#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_navigation)

####################################################################################################
# Project

project(${PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS 
    roscpp
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(
)

####################################################################################################
# Tests