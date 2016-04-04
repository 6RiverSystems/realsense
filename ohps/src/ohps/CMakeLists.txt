cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Debug)

project(ohps)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    OpenCV
    image_transport
    cv_bridge
)

catkin_package()

include_directories(include 
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS} 
    ${image_transport_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS}
)

add_executable(ohps_node 
    ohps.cpp 
    ImageSubscriber.cpp 
    PoseEstimator.cpp
)

target_link_libraries(ohps_node 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES} 
    ${image_transport_LIBRARIES}
    ${cv_bridge_LIBRARIES}
)
