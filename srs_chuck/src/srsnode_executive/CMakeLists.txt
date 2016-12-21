#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_executive)
set(PROJECT_MAIN src/executive_node.cpp)
set(${PROJECT_NAME}_SOURCES

    src/Executive.cpp
    src/task/TaskDetectLabeledAreas.cpp
    src/task/TaskPlaySound.cpp
    src/task/TaskSetMaxVelocity.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    srslib_framework
    srslib_test
)

include_directories(include
    src/
    ${catkin_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(
    DEPENDS srslib_framework
)

add_executable(${PROJECT_NODE} 
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests
