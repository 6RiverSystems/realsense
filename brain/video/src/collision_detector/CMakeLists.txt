cmake_minimum_required(VERSION 2.8)
project(collision_detector)

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED cv_bridge genmsg image_transport sensor_msgs roscpp)

catkin_package()

find_package(OpenCV)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# add the collision detector
add_executable(collision_detector src/collision_detector.cpp)

target_link_libraries(collision_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  /usr/local/lib/librealsense.so
)