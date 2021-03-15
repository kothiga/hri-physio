# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

set(TARGET_NAME physioStreamer)

set(${TARGET_NAME}_SRC
    src/physioStreamer.cpp
)

if( ENABLE_ROS )
    include_directories(/opt/ros/$ENV{ROS_DISTRO}/include /opt/ros/$ENV{ROS_DISTRO}/lib)
endif( ENABLE_ROS )

add_executable(
    ${TARGET_NAME} 
    ${${TARGET_NAME}_SRC}
)

target_link_libraries(
    ${TARGET_NAME} 
    HriPhysio
)

install(
    TARGETS        ${TARGET_NAME}
    DESTINATION    bin  
)

############################################################
