#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_highlevel_behavior)
set(PROJECT_MAIN src/highlevel_behavior_node.cpp)
set(${PROJECT_NAME}_SOURCES

    src/HighLevelBehavior.cpp
    src/costmap_2d/CostMap2d.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    srslib_framework
    srslib_test
)

find_package(GTest REQUIRED)

include_directories(include
    src/
    ${catkin_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
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
