# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

set(TEST_TARGET_NAME test_libHriPhysio_Core)

# Expose doctest.h to cmake.
include_directories(../)

set(${TEST_TARGET_NAME}_SRC
    docTestDefine.cpp
    ringBufferTest.cpp
)

add_executable(
    ${TEST_TARGET_NAME} 
    ${${TEST_TARGET_NAME}_SRC}
)

target_link_libraries(
    ${TEST_TARGET_NAME} 
    HriPhysio
)

############################################################
