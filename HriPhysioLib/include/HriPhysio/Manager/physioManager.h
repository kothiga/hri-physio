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

#ifndef HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H
#define HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H

#include <iostream>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <HriPhysio/Manager/threadManager.h>
#include <HriPhysio/Stream/streamerInterface.h>

#include <HriPhysio/Core/ringBuffer.h>
#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class PhysioManager;
    }
}

class hriPhysio::Manager::PhysioManager : public hriPhysio::Manager::ThreadManager {
private:
    
//
//    double period_read;
//    double period_publish;
//    
//    std::atomic<bool> run_read;
//    std::atomic<bool> run_publish;
//
//    std::thread thread_read;
//    std::thread thread_publish;

    std::string dtype;
    int sampling_rate;
    int num_samples;
    int num_channels;
    int frame_length;
    int sample_overlap;
    int buffer_length;

    hriPhysio::Stream::StreamerInterface* stream_input;
    hriPhysio::Stream::StreamerInterface* stream_output;

    hriPhysio::Core::RingBuffer<hriPhysio::varType> buffer;


public:
    PhysioManager(hriPhysio::Stream::StreamerInterface* input, hriPhysio::Stream::StreamerInterface* output);

    ~PhysioManager();

    void configure(const std::string yaml_file);

    void interactive();

    void wait();


private:
    bool threadInit();

    void inputLoop();

    void outputLoop();

};

#endif /* HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H */
