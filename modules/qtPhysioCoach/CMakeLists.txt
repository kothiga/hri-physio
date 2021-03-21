# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

if( ENABLE_ROS )

find_package( roscpp   REQUIRED )
find_package( std_msgs REQUIRED )

include_directories(/opt/ros/$ENV{ROS_DISTRO}/include /opt/ros/$ENV{ROS_DISTRO}/lib)

set(TARGET_NAME qtPhysioCoach)

set(${TARGET_NAME}_SRC
    src/main.cpp
    src/qtPhysioCoach.cpp
)

set(${TARGET_NAME}_HDR
    include/qtPhysioCoach.h
)

add_executable(
    ${TARGET_NAME} 
    ${${TARGET_NAME}_HDR}
    ${${TARGET_NAME}_SRC}
)

target_include_directories(
    ${TARGET_NAME}
    PRIVATE ./include
)

target_link_libraries(
    ${TARGET_NAME} 
    HriPhysio
    fmt
    yaml-cpp
    -I/opt/ros/$ENV{ROS_DISTRO}/include -L/opt/ros/$ENV{ROS_DISTRO}/lib
    -lroscpp -lrostime -lrosconsole -lroscpp_serialization
)

install(
    TARGETS        ${TARGET_NAME}
    DESTINATION    bin  
)

endif( ENABLE_ROS )

############################################################
