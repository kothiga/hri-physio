/* ================================================================================
 * Copyright: (C) 2020, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Stream/streamerInterface.h>

using namespace hriPhysio::Stream;


StreamerInterface::StreamerInterface() {

}


StreamerInterface::~StreamerInterface() {

}


void StreamerInterface::setName(const std::string name) {
    this->name = name;
    return;
}


std::string StreamerInterface::getName() const {
    return name;
}

