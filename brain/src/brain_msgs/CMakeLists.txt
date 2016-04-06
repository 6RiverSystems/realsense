cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Debug)

set(PROJECT_NAME brain_msgs)
project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    message_generation
)

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
)

add_message_files(
    FILES
    RawOdometry.msg
)
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
)
