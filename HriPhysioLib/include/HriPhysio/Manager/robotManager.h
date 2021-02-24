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

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class RobotManager;
    }
}

class hriPhysio::Manager::RobotManager : public hriPhysio::Manager::ThreadManager {
private:
    
    //bool        log_data;
    //std::string log_name;

    hriPhysio::Social::RobotInterface* robot;

public:
    RobotManager(hriPhysio::Social::RobotInterface* robot);

    ~RobotManager();

    void configure(int argc, char **argv);

    void interactive();


private:
    bool threadInit();

    bool setFunctions(const std::vector< std::string >& input);
    bool getFunctions(const std::vector< std::string >& input);

    //void inputLoop();

};

#endif /* HRI_PHYSIO_MANAGER_ROBOT_MANAGER_H */
