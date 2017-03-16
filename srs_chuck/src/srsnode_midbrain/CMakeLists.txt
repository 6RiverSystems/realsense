#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_midbrain)
set(PROJECT_MAIN src/midbrain_node.cpp)
set(${PROJECT_NAME}_SOURCES
    src/Midbrain.cpp
    src/HardStopReflex.cpp
    src/HeadOnCollisionReflex.cpp
    src/Reflexes.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES
    test/gtest_main.cpp
    test/Test_Reflexes_HardStopReflex.cpp
    test/Test_Reflexes_HeadOnCollisionReflex.cpp
)

####################################################################################################
# Project

project(${PROJECT_NAME})

set(PROJECT_NODE ${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")

#set(Boost_ADDITIONAL_VERSIONS "1.61" "1.61.0")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    dynamic_reconfigure
    polyclipping
    srslib_framework
    srslib_test
    srslib_timing
)

generate_dynamic_reconfigure_options(
    cfg/Reflexes.cfg
)

include_directories(include
    src/
    include/
    ${catkin_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
)

####################################################################################################
# Target

catkin_package(
    DEPENDS
        srslib_framework
        polyclipping
)

add_executable(${PROJECT_NODE}
    ${PROJECT_MAIN}
    ${${PROJECT_NAME}_SOURCES}
)

add_dependencies(${PROJECT_NODE}
    ${PROJECT_NAME}_gencfg
)

add_dependencies(${PROJECT_NODE}
    ${catkin_EXPORTED_TARGETS}
    ${srsnode_midbrain_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Installation

install(TARGETS ${PROJECT_NODE}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY cfg/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

####################################################################################################
# Tests
#
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(Test_Reflexes_HardStopReflex
        ${${PROJECT_NAME}_TEST_SOURCES}
        ${${PROJECT_NAME}_SOURCES}
    )
#    add_dependencies(Test_Reflexes
#        ${catkin_EXPORTED_TARGETS}
#        ${srsnode_midbrain_EXPORTED_TARGETS}
#    )
    target_link_libraries(Test_Reflexes_HardStopReflex
        ${catkin_LIBRARIES}
    )
endif()
