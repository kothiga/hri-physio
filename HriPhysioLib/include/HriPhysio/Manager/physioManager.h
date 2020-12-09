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
    std::size_t sampling_rate;
    std::size_t input_frame;
    std::size_t num_channels;
    std::size_t output_frame;
    std::size_t sample_overlap;
    std::size_t buffer_length;

    hriPhysio::Stream::StreamerInterface* stream_input;
    hriPhysio::Stream::StreamerInterface* stream_output;

    hriPhysio::Core::RingBuffer<hriPhysio::varType> buffer;
    hriPhysio::Core::RingBuffer<double> timestamps;


public:
    PhysioManager(hriPhysio::Stream::StreamerInterface* input, hriPhysio::Stream::StreamerInterface* output);

    ~PhysioManager();

    void configure(const std::string yaml_file);

    void interactive();


private:
    bool threadInit();

    void inputLoop();

    void outputLoop();

};

#endif /* HRI_PHYSIO_MANAGER_PHYSIO_MANAGER_H */
