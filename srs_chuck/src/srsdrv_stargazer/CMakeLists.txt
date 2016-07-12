#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsdrv_stargazer)

set(${PROJECT_NAME}_SOURCES
	src/StarGazerMessageProcessor.cpp
	src/StarGazerPointTransformer.cpp
    src/StarGazerFilter.cpp
    src/StarGazer.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES

)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -Werror ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    tf2_sensor_msgs
    srslib_framework
    srslib_test
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

find_package(Boost REQUIRED)

include_directories(include 
    include/srsdrv_stargazer/
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
    src/stargazer_node.cpp
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${boost_LIBRARIES}
    ${srslib_framework_LIBRARIES}
    yaml-cpp
)

####################################################################################################
# Tests
if (CATKIN_ENABLE_TESTING)

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -Wall -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -g -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "-fprofile-arcs -ftest-coverage")

    catkin_add_gtest(TestSuite_Stargazer
    	test/Test_Stargazer.cpp
        ${${PROJECT_NAME}_SOURCES}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/data
    )

    target_link_libraries(TestSuite_Stargazer
        ${catkin_LIBRARIES}
        ${boost_LIBRARIES}
        ${srslib_framework_LIBRARIES}
        yaml-cpp
    )
endif()