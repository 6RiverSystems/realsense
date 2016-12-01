#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_navigation)

set(${PROJECT_NAME}_SOURCES
    src/global_planner/AStarPotentials.cpp
    src/global_planner/QuadraticCalculator.cpp
    src/global_planner/OrientationFilter.cpp
    src/global_planner/AStarExpansion.cpp
    src/global_planner/PotentialCalculator.cpp
    src/global_planner/GradientPath.cpp
    src/global_planner/GridPath.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES

#    test/TestNavigation_OneWay.cpp
#    test/TestNavigation_WithCostMap.cpp
    test/TestNavigation_Barrett_BigSearch.cpp
    test/TestNavigation_SmallSearch.cpp
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
    nav_core
    costmap_2d
    base_local_planner
    pluginlib
    srslib_framework
    srslib_test
)

generate_dynamic_reconfigure_options(
    cfg/SrsPlanner.cfg
)

include_directories(
    INCLUDE_DIRS
    LIBRARIES
    include
    src
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

####################################################################################################
# Target

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES srslib_global_planner
    DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
        srslib_framework
)

add_library(
    srslib_global_planner src/global_planner/SrsPlannerPotentials.cpp
    ${${PROJECT_NAME}_SOURCES}
)

target_link_libraries(srslib_global_planner
    ${catkin_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

install(TARGETS srslib_global_planner
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY 
    include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(TestSuite_Navigation
        test/TestSuite_Navigation.cpp
        ${${PROJECT_NAME}_TEST_SOURCES}
        ${${PROJECT_NAME}_SOURCES}
        WORKING_DIRECTORY ${TEST_WORKING_DIR}
    )
    add_dependencies(${TestSuite_Navigation}
        ${catkin_EXPORTED_TARGETS}
        ${srsnode_navigation_EXPORTED_TARGETS}
    )
    target_link_libraries(TestSuite_Navigation
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${srslib_framework_LIBRARIES}
    )
endif()