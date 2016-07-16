#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_framework)

set(${PROJECT_NAME}_SOURCES
    src/utils/Logging.cpp
    src/utils/CrashHandler.cpp

    src/io/SerialIO.cpp

    src/planning/pathplanning/grid/GridSolutionItem.cpp
    src/planning/pathplanning/grid/GridSolutionFactory.cpp

    src/robotics/controller/BaseController.cpp
    src/robotics/controller/CMUPathController.cpp

    src/localization/Map.cpp
    src/localization/MapNote.cpp

    src/robotics/controller/BaseController.cpp
    src/robotics/controller/CMUPathController.cpp
)

set(${PROJECT_NAME}_MSGS
    MsgHardwareInfo.msg
    MsgMap.msg
    MsgOperationalState.msg
    MsgPose.msg
    MsgSolution.msg
    MsgSolutionItem.msg
)

# set(${PROJECT_NAME}_SRVS
# )

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

set(TEST_SOURCES_LOCALIZATION
    test/localization/Test_Map_Reconstruction.cpp
)

set(TEST_SOURCES_PLANNING
#    test/planning/pathplanning/Test_Trajectory_BarrettMap.cpp
    test/planning/pathplanning/Test_Trajectory_180ShortTrajectory.cpp
    test/planning/pathplanning/Test_Trajectory_LongStraight.cpp
    test/planning/pathplanning/Test_Trajectory_LongTrajectory.cpp
    test/planning/pathplanning/Test_Trajectory_Map.cpp
    test/planning/pathplanning/Test_Trajectory_ShortStraight.cpp
    test/planning/pathplanning/Test_Trajectory_ShortTrajectory.cpp
    test/planning/pathplanning/Test_Trajectory_SingleRotation.cpp
)

set(TEST_SOURCES_ROBOTICS
    test/robotics/robot/Test_Chuck.cpp
    test/robotics/controller/Test_Yoshizawa.cpp
)

set(TEST_SOURCES_SEARCH
    test/search/Test_AStar_Basic.cpp
    test/search/Test_AStar_Map.cpp
#    test/search/Test_AStar_SamePlaceOnMap.cpp
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

# add_service_files(
#     FILES
#     ${${PROJECT_NAME}_SRVS}
# )

generate_messages(
    DEPENDENCIES std_msgs nav_msgs
)

####################################################################################################
# Target

catkin_package(
    INCLUDE_DIRS src include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS opencv message_runtime std_msgs nav_msgs srslib_test
)

add_library(${PROJECT_LIBRARY}
    ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_LIBRARY}
    ${srslib_framework_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_LIBRARY}
    ${PROJECT_LIBRARIES}
    ${UNWIND_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    file(COPY data DESTINATION "/tmp/${PROJECT_NAME}")

    catkin_add_gtest(TestSuite_Datastructure
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_DATASTRUCTURE}
    )
    target_link_libraries(TestSuite_Datastructure
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Filter
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_FILTER}
    )
    target_link_libraries(TestSuite_Filter
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Graph
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_GRAPH}
    )
    target_link_libraries(TestSuite_Graph
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Localization
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_LOCALIZATION}
    )
    target_link_libraries(TestSuite_Localization
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Planning
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_PLANNING}
    )
    target_link_libraries(TestSuite_Planning
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Robotics
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_ROBOTICS}
    )
    target_link_libraries(TestSuite_Robotics
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Search
        test/TestSuite.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_SEARCH}
    )
    target_link_libraries(TestSuite_Search
        ${PROJECT_LIBRARY}
    )

    catkin_add_gtest(TestSuite_Io
        ${${PROJECT_NAME}_SOURCES}
        ${TEST_SOURCES_IO}
    )
    target_compile_definitions(TestSuite_Io 
        PRIVATE ENABLE_TEST_FIXTURE
    )
    target_link_libraries(TestSuite_Io
        ${PROJECT_LIBRARY}
    )
endif()
