cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_position_estimator)
set(PROJECT_NODE ${PROJECT_NAME}_node)
set(${PROJECT_NAME}_SOURCES

    src/framework/Pose.hpp

    src/tap/odometry/Odometry.hpp
    src/tap/odometry/Odometer.hpp
    src/tap/odometry/RosOdometer.cpp

    src/tap/vel_cmd/VelCmd.hpp
    src/tap/vel_cmd/RosVelCmd.cpp
    
    src/tap/brain_stem_status/RosBrainStemStatus.cpp

    src/PositionEstimator.cpp
    
    src/PEState.hpp
    src/Robot.hpp
    
    src/RobotProfile.hpp
    src/Configuration.hpp
)

project(${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    srslib_framework
    srslib_test
)

find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

catkin_package(
    DEPENDS opencv srslib_framework
)

include_directories(include 
    src/
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS} 
    ${srslib_framework_INCLUDE_DIRS}
    ${srslib_test_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

add_executable(${PROJECT_NODE} 
    src/position_estimator_node.cpp
    ${${PROJECT_NAME}_SOURCES}
)
target_link_libraries(${PROJECT_NODE}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${srslib_framework_LIBRARIES}
)

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(TestSuite_Pe
        test/TestSuite_Pe.cpp
        test/Test_Pe_11ConstantSteps.cpp
        test/Test_Pe_Straight.cpp
        ${${PROJECT_NAME}_SOURCES}
    )
    target_link_libraries(TestSuite_Pe
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${srslib_test_LIBRARIES}
    )
endif()
