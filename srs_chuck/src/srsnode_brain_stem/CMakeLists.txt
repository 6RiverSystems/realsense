#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_brain_stem)
set(PROJECT_NODE ${PROJECT_NAME}_node)
set(${PROJECT_NAME}_SOURCES

    src/SerialIO.cpp
    src/MessageProcessor.cpp
    src/Helper.cpp
)
set(${PROJECT_NAME}_TEST_SOURCES
    test/SerialIO_test.cpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    srslib_framework
    srslib_test
)

find_package(Boost REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
    DEPENDS system_lib srslib_framework
)

include_directories(include 
    src/
    ${catkin_INCLUDE_DIRS}
    ${GTest_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

add_executable(${PROJECT_NODE} 
    src/BrainStem.cpp
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
    catkin_add_gtest(serial-io-test
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_TEST_SOURCES}
    )
    target_link_libraries(serial-io-test
        ${catkin_LIBRARIES}
        pthread
        boost_system
    )
endif()

#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(TestSuite_Pe
#        test/TestSuite_Pe.cpp
#        test/Test_Pe_11ConstantSteps.cpp
#        test/Test_Pe_Straight.cpp
#        ${${PROJECT_NAME}_SOURCES}
#    )
#    target_link_libraries(TestSuite_Pe
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBRARIES}
#        ${srslib_test_LIBRARIES}
#    )
#endif()



























#find_package(
#	catkin
#	REQUIRED COMPONENTS
#	roscpp
#	rospy
#	std_msgs
#	tf
#)

#find_package(
#	Boost
#)

#catkin_package(
#	DEPENDS system_lib
#)

#set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

#include_directories(
#	SYSTEM include ${catkin_INCLUDE_DIRS}
#)

#set(${PROJECT_NAME}_SOURCES
#	src/SerialIO.cpp
#	src/MessageProcessor.cpp
#	src/Helper.cpp
#)

#set(${PROJECT_NAME}_TEST_SOURCES
#    test/SerialIO_test.cpp
#)

#add_executable(brain_stem
#    ${${PROJECT_NAME}_SOURCES}
#    src/BrainStem.cpp
#)

#add_executable(brain_stem_simulator
#	src/BrainStemSimulator.cpp
#)

#target_link_libraries( brain_stem
#	${catkin_LIBRARIES}
#	boost_system
#)

#target_link_libraries( brain_stem_simulator
#	${catkin_LIBRARIES}
#	boost_system
#)

####################################################################################################
# Tests

#if (CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(serial-io-test
#        ${${PROJECT_NAME}_SOURCES}
#        ${${PROJECT_NAME}_TEST_SOURCES}
#    )
#    target_link_libraries(serial-io-test
#    	${catkin_LIBRARIES}
#		pthread
#		boost_system
#	)
#endif()
