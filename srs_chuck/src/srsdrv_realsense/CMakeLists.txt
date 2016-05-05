#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsdrv_realsense)
set(PROJECT_MAIN src/realsense_driver.cpp)
set(${PROJECT_NAME}_SOURCES
    src/RealsenseDriver.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES
)
####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    cv_bridge
)

find_package(GTest REQUIRED)

find_package(OpenCV 1.7 REQUIRED)

find_package(PCL REQUIRED)

include_directories(include 
    src/
    ${catkin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(

)

add_executable(${PROJECT_NODE} 
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(TestSuite_RealsenseDriver
        ${${PROJECT_NAME}_TEST_SOURCES}
        ${${PROJECT_NAME}_SOURCES}
    )
    target_link_libraries(TestSuite_RealsenseDriver
        ${catkin_LIBRARIES}
    )
endif()
