/* ================================================================================
 * Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
 *     University of Waterloo, All rights reserved.
 * 
 * Authors: 
 *     Austin Kothig <austin.kothig@uwaterloo.ca>
 * 
 * CopyPolicy: Released under the terms of the BSD 3-Clause License. 
 *     See the accompanying LICENSE file for details.
 * ================================================================================
 */

#include <HriPhysio/Factory/streamerFactory.h>

using namespace hriPhysio::Factory;


StreamerFactory::StreamerFactory() {

}

StreamerFactory::~StreamerFactory() {

}

hriPhysio::Stream::StreamerInterface* StreamerFactory::getStreamer(std::string streamerType) {

    hriPhysio::toUpper(streamerType);
    if (streamerType == "") {
        return nullptr;
    }

    if (streamerType == "LSL") {
        return new hriPhysio::Stream::LslStreamer();
    }

    if (streamerType == "ROS") {
        return nullptr;
    }

    if (streamerType == "YARP") {
        return nullptr;
    }

    return nullptr;
}
