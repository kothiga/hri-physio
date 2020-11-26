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

#ifndef HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H
#define HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Stream {
        class StreamerInterface;
    }
}

class hriPhysio::Stream::StreamerInterface {

private:
    std::string name;

public:
    StreamerInterface();

    ~StreamerInterface();

    void setName(const std::string name);

    std::string getName() const;

    virtual void publish(const std::vector<hriPhysio::varType>&  buff, std::size_t channels) = 0;
    virtual void receive(std::vector<hriPhysio::varType>& buff, std::size_t channels) = 0;

private:
    void tempfunc();

};

#endif /* HRI_PHYSIO_STREAM_STREAMER_INTERFACE_H */
