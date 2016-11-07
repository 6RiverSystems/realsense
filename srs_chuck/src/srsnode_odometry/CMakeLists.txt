#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_odometry)
set(PROJECT_MAIN src/odometry_node.cpp)
set(${PROJECT_NAME}_SOURCES
    src/OdometryPositionEstimator.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")

#set(Boost_ADDITIONAL_VERSIONS "1.61" "1.61.0")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    dynamic_reconfigure
    srslib_framework
    srslib_test
)

include_directories(include
    include/
    ${catkin_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/RobotSetup.cfg
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

add_dependencies(${PROJECT_NODE}
    ${catkin_EXPORTED_TARGETS}
    ${srsnode_midbrain_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

endif()
