#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsdrv_brainstem)
set(PROJECT_MAIN src/BrainStem.cpp)
set(${PROJECT_NAME}_SOURCES

    src/BrainStemMessageProcessor.cpp
    src/BrainStem.cpp
	src/BrainStemEmulator.cpp
	
    src/hw_message/SensorFrameHandler.cpp
    src/hw_message/RawOdometryHandler.cpp
    src/hw_message/HardwareInfoHandler.cpp
    
    src/sw_message/SoundHandler.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES

)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    srslib_framework
    srslib_test
)

find_package(Boost REQUIRED)

include_directories(include 
    include/srsdrv_brainstem/
    src/
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(
    DEPENDS system_lib srslib_framework
)

add_executable(${PROJECT_NODE} 
    src/brainstem_node.cpp
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${boost_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(TestSuite_Brainstem
    	test/Test_MessageProcessor.cpp
        ${${PROJECT_NAME}_SOURCES}
    )
    target_link_libraries(TestSuite_Brainstem
        ${catkin_LIBRARIES}
        ${boost_LIBRARIES}
        ${srslib_framework_LIBRARIES}
    )
endif()