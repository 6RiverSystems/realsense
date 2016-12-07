#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_framework)

set(${PROJECT_NAME}_SOURCES
    src/datastructure/graph/grid2d/Grid2d.cpp

    src/io/SerialIO.cpp

    src/localization/map/BaseMap.cpp
    src/localization/map/MapAdapter.cpp
    src/localization/map/MapNote.cpp
    src/localization/map/MapStack.cpp
    src/localization/map/MapStackFactory.cpp
    src/localization/map/mapnote/MapNote2.cpp
    src/localization/map/mapnote/MapNotes.cpp
    src/localization/map/mapnote/WarningSound.cpp
    src/localization/map/logical/LogicalMap.cpp
    src/localization/map/logical/LogicalMapFactory.cpp
    src/localization/map/occupancy/OccupancyMap.cpp
    src/localization/map/occupancy/OccupancyMapFactory.cpp

    src/planning/pathplanning/grid2d/Grid2dSolutionFactory.cpp
    src/planning/pathplanning/grid2d/Grid2dSolutionItem.cpp
    src/planning/pathplanning/grid2d/Grid2dSolutionUtils.cpp

    src/platform/CrashHandler.cpp

    src/robotics/controller/BaseController.cpp
    src/robotics/controller/CMUPathController.cpp

    src/ros/message/LogicalMapMessageFactory.cpp

    src/search/AStar.cpp
    src/search/graph/grid2d/Grid2dAction.cpp
    src/search/graph/grid2d/Grid2dNode.cpp
    src/search/graph/mapstack/MapStackAction.cpp
    src/search/graph/mapstack/MapStackNode.cpp

    src/utils/Logging.cpp
)

set(${PROJECT_NAME}_MSGS
    Imu.msg
    JoypadState.msg
    LogicalArea.msg
    LogicalCell.msg
    LogicalMap.msg
    LogicalMetadata.msg
    MapNote.msg
    MapStack.msg
    MsgHardwareInfo.msg
    MsgOperationalState.msg
    MsgSolution.msg
    MsgSolutionItem.msg
    OccupancyMap.msg
    OccupancyMetadata.msg
    Pose.msg
    SensorFrame.msg
    Sound.msg
    Velocity.msg
    Odometry.msg
    OdometryRPM.msg
)

set(${PROJECT_NAME}_SRVS
    ExecuteSolution.srv
)

set(TEST_SOURCES_DATASTRUCTURE
    test/datastructure/graph/Test_Grid2d_Basic.cpp
    test/datastructure/queue/Test_MappedPriorityQueue_Basic.cpp
    test/datastructure/queue/Test_MappedPriorityQueue_IndexClass.cpp
    test/datastructure/queue/Test_MappedPriorityQueue_InnerHashEqual.cpp
    test/datastructure/queue/Test_MappedPriorityQueue_PtrIndexClass.cpp
    test/datastructure/queue/Test_MappedPriorityQueue_UserDefinedClass.cpp
)

set(TEST_SOURCES_FILTER
    test/filter/ukf/Test_Ukf_Basic.cpp
)

set(TEST_SOURCES_LOCALIZATION
    test/localization/Test_Empty_BasicRead.cpp
    test/localization/Test_MapNote_Basic.cpp
    test/localization/Test_MicroMap_BasicRead.cpp
    test/localization/Test_MicroMap_MapNote.cpp
    test/localization/Test_MicroMap_WithAisles.cpp
    test/localization/Test_SmallMap_BasicRead.cpp
)

set(TEST_SOURCES_MATH
    test/math/Test_AngleMath.cpp
    test/math/Test_BasicMath.cpp
    test/math/Test_PoseMath.cpp
)

set(TEST_SOURCES_PLANNING
    test/planning/pathplanning/grid2d/Test_Solution_Grid2d_Basic.cpp
    test/planning/pathplanning/grid2d/Test_Solution_Grid2d_Empty_Consecutive.cpp
    test/planning/pathplanning/grid2d/Test_Solution_MapStack_Basic.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_180ShortTrajectory.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_LongStraight.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_LongTrajectory.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_ShortStraight.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_ShortTrajectory.cpp
    test/planning/pathplanning/trajectory/Test_Trajectory_SingleRotation.cpp
)

set(TEST_SOURCES_PLATFORM
    test/platform/Test_CrashHandler.cpp
)

set(TEST_SOURCES_ROBOTICS
    test/robotics/robot/Test_Chuck.cpp
)

set(TEST_SOURCES_ROS
    test/ros/message/Test_LogicalMap_Reconstruction.cpp
    test/ros/message/Test_OccupancyMap_Reconstruction.cpp
)

set(TEST_SOURCES_SEARCH
    test/search/grid2d/Test_AStar_Grid2d_6rshq.cpp
    test/search/grid2d/Test_AStar_Grid2d_Barrett.cpp
    test/search/grid2d/Test_AStar_Grid2d_Basic.cpp
    test/search/grid2d/Test_AStar_Grid2d_NoSolution.cpp
    test/search/grid2d/Test_AStar_Grid2d_SamePlaceOnMap.cpp
    test/search/grid2d/Test_AStar_Grid2d_Weights.cpp
    test/search/mapstack/Test_AStar_MapStack_Barrett.cpp
    test/search/mapstack/Test_AStar_MapStack_Basic.cpp
    test/search/mapstack/Test_AStar_MapStack_NoSolution.cpp
    test/search/mapstack/Test_AStar_MapStack_SamePlaceOnMap.cpp
    test/search/mapstack/Test_AStar_MapStack_Unknown.cpp
    test/search/mapstack/Test_AStar_MapStack_Weights.cpp
)

set(TEST_SOURCES_IO
    test/io/Test_SerialIO.cpp
)
####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_LIBRARY ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")
# set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    message_generation
    std_msgs
    nav_msgs
    costmap_2d
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(srslib_test REQUIRED)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
pkg_check_modules(UNWIND libunwind)

include_directories(
    include/
    src/
    test/
    ${roscpp_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS} 
    ${srslib_test_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIBRARIES
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${srslib_test_LIBRARIES}
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
    DEPENDENCIES 
        std_msgs 
        nav_msgs
)

####################################################################################################

# Target

catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS 
        opencv 
        message_runtime 
        std_msgs 
        nav_msgs 
        srslib_test
        costmap_2d
)

add_library(${PROJECT_LIBRARY}
    ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_LIBRARY}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_LIBRARY}
    ${PROJECT_LIBRARIES}
    ${UNWIND_LIBRARIES}
    ${catkin_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(TestSuite_Datastructure
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_DATASTRUCTURE}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/datastructure
    )
    target_link_libraries(TestSuite_Datastructure
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Filter
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_FILTER}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/filter
    )
    target_link_libraries(TestSuite_Filter
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Math
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_MATH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/math
    )
    target_link_libraries(TestSuite_Math
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Localization
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_LOCALIZATION}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/localization
    )
    target_link_libraries(TestSuite_Localization
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Planning
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_PLANNING}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/planning
    )
    target_link_libraries(TestSuite_Planning
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Platform
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_PLATFORM}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/platform
    )
    target_link_libraries(TestSuite_Platform
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Robotics
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_ROBOTICS}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/robotics
    )
    target_link_libraries(TestSuite_Robotics
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Ros
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_ROS}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/ros
    )
    target_link_libraries(TestSuite_Ros
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Search
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_SEARCH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/search
    )
    target_link_libraries(TestSuite_Search
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Io
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_IO}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/io
    )
    target_compile_definitions(TestSuite_Io 
        PRIVATE ENABLE_TEST_FIXTURE
    )
    target_link_libraries(TestSuite_Io
        ${PROJECT_LIBRARY}
    )
endif()
