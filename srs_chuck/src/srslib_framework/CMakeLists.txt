#
# (c) Copyright 2015-2016 River Systems, all rights reserved.
#
# This is proprietary software, unauthorized distribution is not permitted.
#
cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srslib_framework)
set(PROJECT_LIBRARY ${PROJECT_NAME})
set(${PROJECT_NAME}_SOURCES

    include/${PROJECT_NAME}/filter/Command.hpp
    include/${PROJECT_NAME}/filter/FilterState.hpp
    include/${PROJECT_NAME}/filter/Measurement.hpp
    include/${PROJECT_NAME}/filter/Process.hpp
    include/${PROJECT_NAME}/filter/Sensor.hpp

    include/${PROJECT_NAME}/filter/ukf/UnscentedKalmanFilter.hpp

    include/${PROJECT_NAME}/platform/Object.hpp
    include/${PROJECT_NAME}/platform/ThreadSafeQueue.hpp

    include/${PROJECT_NAME}/math/Math.hpp
    
    include/${PROJECT_NAME}/robotics/Pose.hpp
    include/${PROJECT_NAME}/robotics/Odometry.hpp
    include/${PROJECT_NAME}/robotics/Velocity.hpp

    include/${PROJECT_NAME}/ros/RosTap.hpp
    include/${PROJECT_NAME}/ros/tap/RosTapCmdVel.hpp
    include/${PROJECT_NAME}/ros/tap/RosTapJoy.hpp

    include/${PROJECT_NAME}/utils/Print.hpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
)
find_package(OpenCV REQUIRED)
find_package(srslib_test REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_LIBRARY}
    DEPENDS opencv
)

include_directories(
    include/
    test/
    ${roscpp_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${srslib_test_INCLUDE_DIRS}
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

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(TestSuite_Ukf
        test/ukf/TestSuite_Ukf.cpp
        test/ukf/Test_Ukf_Basic.cpp
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_TEST_SOURCES}
    )
    target_link_libraries(TestSuite_Ukf
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )
endif()
