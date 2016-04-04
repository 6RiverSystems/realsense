cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Debug)

project(position_estimator)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    brain_msgs
#    OpenCV
)

include_directories(include 
    ${catkin_INCLUDE_DIRS}
    src/
#    ${OpenCV_INCLUDE_DIRS} 
)

add_executable(position_estimator_node

    src/platform/ThreadSafeQueue.hpp
    
    src/framework/SensorReading.hpp
    src/framework/SensorFrameQueue.cpp
    
    src/sensor/Sensor.hpp
    src/sensor/OdometrySensor.cpp

    src/position_estimator_node.cpp
    src/PositionEstimator.cpp
)

target_link_libraries(position_estimator_node 
    ${catkin_LIBRARIES} 
#    ${OpenCV_LIBRARIES} 
)
