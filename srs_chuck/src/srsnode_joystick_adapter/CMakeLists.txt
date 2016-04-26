cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_joystick_adapter)
set(PROJECT_NODE ${PROJECT_NAME}_node)
set(${PROJECT_NAME}_SOURCES

    src/${PROJECT_NAME}/JoystickAdapter.cpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    srslib_framework
    srslib_test
)

find_package(GTest REQUIRED)

catkin_package(
    DEPENDS srslib_framework
)

include_directories(include
    src/
    ${catkin_INCLUDE_DIRS}
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NODE} 
    src/${PROJECT_NAME}/joystick_adapter_node.cpp
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests
