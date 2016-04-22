cmake_minimum_required(VERSION 2.8.3)

set(PROJECT_NAME srsnode_brain_stem)

project(${PROJECT_NAME})

find_package(
	catkin
	REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	tf
)

find_package(
	Boost
)

catkin_package(
	DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread ${CMAKE_CXX_FLAGS}")

include_directories(
	SYSTEM include ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SOURCES
	src/SerialIO.cpp
	src/MessageProcessor.cpp
	src/Helper.cpp
)

set(${PROJECT_NAME}_TEST_SOURCES
    test/SerialIO_test.cpp
)

add_executable(brain_stem
    ${${PROJECT_NAME}_SOURCES}
    src/BrainStem.cpp
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

####################################################################################################
# Tests

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(serial-io-test
        ${${PROJECT_NAME}_SOURCES}
        ${${PROJECT_NAME}_TEST_SOURCES}
    )
    target_link_libraries(serial-io-test
    	${catkin_LIBRARIES}
		pthread
		boost_system
	)
endif()
