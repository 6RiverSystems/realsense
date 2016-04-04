cmake_minimum_required(VERSION 2.8.3)
project(video)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    brain_msgs
)