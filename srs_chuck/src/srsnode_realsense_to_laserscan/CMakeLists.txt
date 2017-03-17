#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_realsense_to_laserscan)
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

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    sensor_msgs
    message_filters
    nodelet
)

find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include 
    src/
    include/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(

)

add_library(RealsenseDriver src/RealsenseDriver.cpp)
target_link_libraries(RealsenseDriver ${catkin_LIBRARIES})

add_library(RealsenseDriverNodelet src/RealsenseDriverNodelet.cpp)
target_link_libraries(RealsenseDriverNodelet RealsenseDriver ${catkin_LIBRARIES})

add_executable(${PROJECT_NODE} 
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${cv_bridge_LIBRARIES}
)

####################################################################################################
# Installation

install(TARGETS ${PROJECT_NODE}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES nodelets.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodelets.xml
)

####################################################################################################
# Tests
#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(TestSuite_RealsenseDriver
#        ${${PROJECT_NAME}_TEST_SOURCES}
#        ${${PROJECT_NAME}_SOURCES}
#    )
#    target_link_libraries(TestSuite_RealsenseDriver
#        ${catkin_LIBRARIES}
#    )
#endif()
