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

    src/hw_message/LogHandler.cpp
    src/hw_message/ButtonPressedHandler.cpp
    src/hw_message/OdometryRpmHandler.cpp
    src/hw_message/HardwareInfoHandler.cpp
    src/hw_message/OperationalStateHandler.cpp
    src/hw_message/PowerStateHandler.cpp

    src/sw_message/StartupHandler.cpp
    src/sw_message/ShutdownHandler.cpp
    src/sw_message/PingHandler.cpp
    src/sw_message/FreeSpinHandler.cpp
    src/sw_message/SetOdometryRpmHandler.cpp
    src/sw_message/SoundHandler.cpp
    src/sw_message/SetMotionStateHandler.cpp
    src/sw_message/UpdateUIHandler.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES
	test/hw_message/Test_Log.cpp
    test/hw_message/Test_ButtonPressed.cpp
    test/hw_message/Test_OdometryRpm.cpp
    test/hw_message/Test_HardwareInfo.cpp
    test/hw_message/Test_OperationalState.cpp
    test/hw_message/Test_PowerState.cpp
    test/TestSuite_Brainstem.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    srslib_framework
    srslib_test
)

find_package(Boost REQUIRED)

include_directories(include 
    include/srsdrv_brainstem
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
	include_directories(test 
	)

    catkin_add_gtest(TestSuite_Brainstem
        ${${PROJECT_NAME}_TEST_SOURCES}
        ${${PROJECT_NAME}_SOURCES}
    )
    target_link_libraries(TestSuite_Brainstem
        ${catkin_LIBRARIES}
        ${boost_LIBRARIES}
        ${srslib_framework_LIBRARIES}
    )

endif()