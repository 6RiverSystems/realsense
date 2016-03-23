cmake_minimum_required(VERSION 2.8.3)
project(brain)

find_package(catkin REQUIRED COMPONENTS
	rospy
	std_msgs
	geometry_msgs
	message_generation
)

add_message_files(
	FILES
	RawOdometry.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)