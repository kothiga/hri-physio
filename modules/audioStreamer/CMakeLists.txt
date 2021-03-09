# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

if( ENABLE_ROS )

set(TARGET_NAME audioStreamer)

set(${TARGET_NAME}_PYTHON
    python/audioPlayer.py
    python/audioStreamer.py
)

install(
    FILES          ${${TARGET_NAME}_PYTHON}
    DESTINATION    bin  
)

endif( ENABLE_ROS )

############################################################
