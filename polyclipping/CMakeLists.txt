CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
PROJECT(polyclipping)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release type")

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES polyclipping)

include_directories(include/ include/polyclipping)

ADD_LIBRARY(polyclipping src/clipper.cpp)

SET_TARGET_PROPERTIES(${PROJECT_LIBRARY} PROPERTIES VERSION 22.0.0 SOVERSION 22 )

####################################################################################################
# Installation

install(TARGETS polyclipping
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)