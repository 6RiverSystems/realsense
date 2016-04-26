cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_test)
set(PROJECT_LIBRARY ${PROJECT_NAME})
set(${PROJECT_NAME}_SOURCES

    include/srslib_test/utils/Compare.hpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS opencv
)

include_directories(
    include/
    ${roscpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
)

add_library(${PROJECT_LIBRARY}
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_LIBRARY}
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

set_target_properties(${PROJECT_LIBRARY} PROPERTIES LINKER_LANGUAGE CXX)

####################################################################################################
# Tests
