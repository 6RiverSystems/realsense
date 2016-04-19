cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME position_estimator)
set(PROJECT_NODE ${PROJECT_NAME}_node)
set(${PROJECT_NAME}_SOURCES
    src/platform/ThreadSafeQueue.hpp
    src/platform/Object.hpp
    
    src/framework/Math.hpp
    src/framework/Utils.hpp
    src/framework/Pose.hpp

    src/filter/FilterState.hpp
    src/filter/Command.hpp
    src/filter/Process.hpp
    src/filter/Measurement.hpp
    src/filter/Sensor.hpp
    src/filter/ukf/UnscentedKalmanFilter.hpp

#    src/sensor/MeasurementQueue.hpp
    
    src/sensor/odometry/Odometry.hpp
    src/sensor/odometry/Odometer.hpp
    src/sensor/odometry/RosOdometer.cpp

    src/PositionEstimator.cpp
    src/PEState.hpp
    src/VelCmd.hpp
    src/RobotProfile.hpp
    src/Configuration.hpp
    src/Robot.hpp
)

set(${PROJECT_NAME}_TEST_SOURCES
    test/framework/Compare.hpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    brain_msgs
)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
    INCLUDE_DIRS src test
    LIBRARIES ${PROJECT_NAME}
    CATKIN-DEPENDS std_msgs brain_msgs
    DEPENDS opencv
)

include_directories(include 
    src/
    test/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NODE} 
    src/position_estimator_node.cpp
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NODE} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(UkfTest
        test/ukf/UkfTest.cpp
        test/ukf/UkfTest_Basic.cpp
        test/ukf/UkfTest_11ConstantSteps.cpp 
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_TEST_SOURCES}
    )
    target_link_libraries(UkfTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endif()
