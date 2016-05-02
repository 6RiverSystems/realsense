#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_position_estimator)
set(PROJECT_MAIN src/position_estimator_node.cpp)
set(${PROJECT_NAME}_SOURCES

    include/${PROJECT_NAME}/tap/aps/ApsSensor.hpp
    src/tap/aps/RosTapAps.cpp

    include/${PROJECT_NAME}/tap/brain_stem_status/RosTapBrainStemStatus.hpp

    include/${PROJECT_NAME}/tap/initial_pose/RosTapInitialPose.hpp

    include/${PROJECT_NAME}/tap/odometry/OdometrySensor.hpp
    include/${PROJECT_NAME}/tap/odometry/RosTapOdometry.hpp

    include/${PROJECT_NAME}/CmdVelocity.hpp
    include/${PROJECT_NAME}/Configuration.hpp
    include/${PROJECT_NAME}/PositionEstimator.hpp
    include/${PROJECT_NAME}/Robot.hpp
    include/${PROJECT_NAME}/RobotProfile.hpp
    include/${PROJECT_NAME}/StatePe.hpp

    src/PositionEstimator.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES

    test/Test_Pe_11ConstantSteps.cpp
    test/Test_Pe_Straight.cpp
)
####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    srslib_framework
    srslib_test
)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

include_directories(include 
    src/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(
    DEPENDS opencv srslib_framework
)

add_executable(${PROJECT_NODE} 
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(TestSuite_Pe
        test/TestSuite_Pe.cpp
        ${${PROJECT_NAME}_TEST_SOURCES}
        ${${PROJECT_NAME}_SOURCES}
    )
    target_link_libraries(TestSuite_Pe
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )
endif()
