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

#ifndef HRI_PHYSIO_STREAM_ROS_STREAMER_H
#define HRI_PHYSIO_STREAM_ROS_STREAMER_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <HriPhysio/Stream/streamerInterface.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Stream {
        class RosStreamer;
    }
}

class hriPhysio::Stream::RosStreamer : public hriPhysio::Stream::StreamerInterface {

private:
    int temp;

public:
    RosStreamer();

    ~RosStreamer();

//    lsl::channel_format_t getLslFormatType();

    bool openInputStream();

    bool openOutputStream();

    void publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps=nullptr);
    
    void receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps=nullptr);

private:
    template<typename T>
    void pushStream(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps);

    template<typename T>
    void pullStream(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps);
    
};

#endif /* HRI_PHYSIO_STREAM_ROS_STREAMER_H */
