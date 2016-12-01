CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
PROJECT(polyclipping)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release type")

catkin_package(LIBRARIES polyclipping)

ADD_LIBRARY(polyclipping clipper.cpp)

SET_TARGET_PROPERTIES(polyclipping PROPERTIES VERSION 22.0.0 SOVERSION 22 )
