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

#ifndef HRI_PHYSIO_MANAGER_ROBOT_MANAGER_H
#define HRI_PHYSIO_MANAGER_ROBOT_MANAGER_H

#include <iostream>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>

#include <yaml-cpp/yaml.h>

#include <HriPhysio/Manager/threadManager.h>
#include <HriPhysio/Social/robotInterface.h>
#include <HriPhysio/Stream/streamerInterface.h>
#include <HriPhysio/Stream/csvStreamer.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class RobotManager;
    }
}

class hriPhysio::Manager::RobotManager : public hriPhysio::Manager::ThreadManager {
private:
    
    bool        log_data;
    std::string log_name;

    hriPhysio::Stream::CsvStreamer robot_logger;
    hriPhysio::Social::RobotInterface* robot;

    std::chrono::_V2::system_clock::time_point start_time;

public:
    RobotManager(hriPhysio::Social::RobotInterface* robot);

    ~RobotManager();

    void configure(int argc, char **argv);

    void interactive();


private:
    bool threadInit();

    void process(const std::string& inp);
    bool setFunctions(const std::vector< std::string >& input);
    bool getFunctions(const std::vector< std::string >& input);

    void selfLoop();
    void inputLoop();

};

#endif /* HRI_PHYSIO_MANAGER_ROBOT_MANAGER_H */
