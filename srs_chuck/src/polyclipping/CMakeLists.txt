CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
PROJECT(polyclipping)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release type")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polyclipping)

include_directories(include/ include/polyclipping)

ADD_LIBRARY(polyclipping src/clipper.cpp)

SET_TARGET_PROPERTIES(polyclipping PROPERTIES VERSION 22.0.0 SOVERSION 22 )
