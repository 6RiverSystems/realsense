#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_framework)

set(${PROJECT_NAME}_SOURCES

    include/${PROJECT_NAME}/datastructure/ThreadSafeQueue.hpp
    include/${PROJECT_NAME}/datastructure/MappedPriorityQueue.hpp

    include/${PROJECT_NAME}/filter/Command.hpp
    include/${PROJECT_NAME}/filter/FilterState.hpp
    include/${PROJECT_NAME}/filter/Measurement.hpp
    include/${PROJECT_NAME}/filter/Process.hpp
    include/${PROJECT_NAME}/filter/Sensor.hpp

    include/${PROJECT_NAME}/filter/ukf/UnscentedKalmanFilter.hpp

    include/${PROJECT_NAME}/graph/grid2d/Grid2dLocation.hpp
    include/${PROJECT_NAME}/graph/grid2d/Grid2dNode.hpp
    include/${PROJECT_NAME}/graph/grid2d/Grid2d.hpp

    include/${PROJECT_NAME}/io/SerialIO.hpp
    src/io/SerialIO.cpp

    include/${PROJECT_NAME}/localization/Anchor.hpp
    src/localization/Map.cpp
    
    include/${PROJECT_NAME}/math/Math.hpp
    include/${PROJECT_NAME}/math/Time.hpp
    include/${PROJECT_NAME}/math/PoseMath.hpp

    include/${PROJECT_NAME}/planning/pathplanning/Trajectory.hpp
    include/${PROJECT_NAME}/planning/pathplanning/SolutionNode.hpp

    include/${PROJECT_NAME}/platform/Object.hpp
    include/${PROJECT_NAME}/platform/Thread.hpp

    include/${PROJECT_NAME}/robotics/Pose.hpp
    include/${PROJECT_NAME}/robotics/Odometry.hpp
    include/${PROJECT_NAME}/robotics/Velocity.hpp

    include/${PROJECT_NAME}/robotics/controller/YoshizawaController.hpp

    include/${PROJECT_NAME}/ros/RosTap.hpp
    include/${PROJECT_NAME}/ros/RosService.hpp
    include/${PROJECT_NAME}/ros/service/RosServiceTrigger.hpp
    include/${PROJECT_NAME}/ros/service/RosTriggerShutdown.hpp
    include/${PROJECT_NAME}/ros/tap/RosTapCmdVel.hpp
    include/${PROJECT_NAME}/ros/tap/RosTapJoy.hpp
    include/${PROJECT_NAME}/ros/tap/RosTapJoyAdapter.hpp
    src/ros/tap/RosTapMap.cpp

    include/${PROJECT_NAME}/search/SearchAction.hpp
    include/${PROJECT_NAME}/search/SearchPosition.hpp
    include/${PROJECT_NAME}/search/SearchNode.hpp
    include/${PROJECT_NAME}/search/AStar.hpp
    src/search/SearchPositionNote.cpp

    include/${PROJECT_NAME}/utils/Ocv2Base.hpp
    include/${PROJECT_NAME}/utils/Filesystem.hpp
)

set(${PROJECT_NAME}_MSGS
    Aps.msg
    CompleteMap.msg
)

set(${PROJECT_NAME}_SRVS
    MapCoordinates.srv
)

set(TEST_SOURCES_DATASTRUCTURE
    test/datastructure/Test_MappedPriorityQueue_Basic.cpp
    test/datastructure/Test_MappedPriorityQueue_UserDefinedKey.cpp
)

set(TEST_SOURCES_FILTER
    test/filter/ukf/Test_Ukf_Basic.cpp
)

set(TEST_SOURCES_GRAPH
    test/graph/Test_Grid2d_Basic.cpp
)

set(TEST_SOURCES_PLANNING
    test/planning/pathplanning/Test_Trajectory_LongTrajectory.cpp
    test/planning/pathplanning/Test_Trajectory_ShortStraight.cpp
    test/planning/pathplanning/Test_Trajectory_ShortTrajectory.cpp
)

set(TEST_SOURCES_ROBOTICS
    test/robotics/robot/Test_Chuck.cpp
    test/robotics/controller/Test_Yoshizawa.cpp
)

set(TEST_SOURCES_SEARCH
    test/search/Test_AStar_Basic.cpp
    test/search/Test_AStar_Map.cpp
)

set(TEST_SOURCES_IO
    test/io/Test_SerialIO.cpp
)
####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_LIBRARY ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    message_generation
    std_msgs
    nav_msgs
)
find_package(OpenCV REQUIRED)
find_package(srslib_test REQUIRED)
find_package(Boost REQUIRED)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)

include_directories(
    include/
    src/
    test/
    ${roscpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${srslib_test_INCLUDE_DIRS}
)

set(PROJECT_LIBRARIES
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    yaml-cpp
    SDL SDL_image
)

####################################################################################################
# Messages & Services

add_message_files(
    FILES
    ${${PROJECT_NAME}_MSGS}
)

add_service_files(
    FILES
    ${${PROJECT_NAME}_SRVS}
)

generate_messages(
    DEPENDENCIES std_msgs nav_msgs
)

####################################################################################################
# Target

catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS opencv message_runtime std_msgs nav_msgs
)

add_library(${PROJECT_LIBRARY}
    ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_LIBRARY}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_LIBRARY}
    ${PROJECT_LIBRARIES}
)

set_target_properties(${PROJECT_LIBRARY} 
    PROPERTIES LINKER_LANGUAGE CXX
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(TestSuite_Datastructure
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_DATASTRUCTURE}
    )
    target_link_libraries(TestSuite_Datastructure
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Filter
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_FILTER}
    )
    target_link_libraries(TestSuite_Filter
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Graph
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_GRAPH}
    )
    target_link_libraries(TestSuite_Graph
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Planning
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_PLANNING}
    )
    target_link_libraries(TestSuite_Planning
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Robotics
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_ROBOTICS}
    )
    target_link_libraries(TestSuite_Robotics
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Search
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_SEARCH}
    )
    target_link_libraries(TestSuite_Search
        ${catkin_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )

    catkin_add_gtest(TestSuite_Io
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_IO}
    )
    
    target_compile_definitions(TestSuite_Io 
        PRIVATE ENABLE_TEST_FIXTURE
    )

    target_link_libraries(TestSuite_Io
        ${catkin_LIBRARIES}
        ${srslib_test_LIBRARIES}
        ${PROJECT_LIBRARIES}
        ${Boost_LIBRARIES}
    )
endif()
