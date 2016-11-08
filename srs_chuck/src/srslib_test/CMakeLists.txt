#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_test)

set(${PROJECT_NAME}_SOURCES
    src/utils/MemoryWatch.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_LIBRARY ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
)
find_package(OpenCV REQUIRED)

include_directories(
    include/
    ${roscpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

####################################################################################################
# Target

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS opencv
)

add_library(${PROJECT_LIBRARY}
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(${PROJECT_LIBRARY}
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

####################################################################################################
# Tests
