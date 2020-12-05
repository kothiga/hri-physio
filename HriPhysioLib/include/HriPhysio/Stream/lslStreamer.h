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

#ifndef HRI_PHYSIO_STREAM_LSL_STREAMER_H
#define HRI_PHYSIO_STREAM_LSL_STREAMER_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <HriPhysio/Stream/streamerInterface.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Stream {
        class LslStreamer;
    }
}

class hriPhysio::Stream::LslStreamer : public hriPhysio::Stream::StreamerInterface {

private:
    std::string temp;

public:
    LslStreamer();

    ~LslStreamer();

    bool openInputStream();
    bool openOutputStream();

    void publish(const std::vector<hriPhysio::varType>&  buff, std::size_t channels);
    void receive(std::vector<hriPhysio::varType>& buff, std::size_t channels);

private:
    template<typename T>
    void pushStream(const std::vector<hriPhysio::varType>&  buff, std::size_t channels);

    template<typename T>
    void pullStream(std::vector<hriPhysio::varType>& buff, std::size_t channels);
    
};

#endif /* HRI_PHYSIO_STREAM_LSL_STREAMER_H */
