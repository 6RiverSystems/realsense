cmake_minimum_required(VERSION 2.8.3)
project(brain_stem)

find_package(
	catkin
	REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	brain_msgs
	tf
)

find_package(
	Boost
)

catkin_package(
	DEPENDS system_lib
    CATKIN-DEPENDS brain_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

include_directories(
	SYSTEM include ${catkin_INCLUDE_DIRS}
)

add_executable(brain_stem
	src/BrainStem.cpp
	src/SerialIO.cpp
	src/MessageProcessor.cpp
	src/Helper.cpp
)

add_executable(brain_stem_simulator
	src/BrainStemSimulator.cpp
)

target_link_libraries( brain_stem
	${catkin_LIBRARIES}
	boost_system
)

target_link_libraries( brain_stem_simulator
	${catkin_LIBRARIES}
	boost_system
)
 
catkin_add_gtest(serial-io-test src/SerialIO.cpp test/SerialIO_test.cpp)
target_link_libraries(serial-io-test ${GTEST_LIBRARIES} pthread boost_system)
