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

#ifndef HRI_PHYSIO_STREAM_CSV_STREAMER_H
#define HRI_PHYSIO_STREAM_CSV_STREAMER_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ctime>
#include <iomanip>
#include <fstream>

#include <HriPhysio/Stream/streamerInterface.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Stream {
        class CsvStreamer;
    }
}

class hriPhysio::Stream::CsvStreamer : public hriPhysio::Stream::StreamerInterface {

private:
    std::ifstream input;
    std::ofstream output;
    std::string sys_time;


public:
    CsvStreamer();

    ~CsvStreamer();

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

#endif /* HRI_PHYSIO_STREAM_CSV_STREAMER_H */
