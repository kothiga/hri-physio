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

#ifndef HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H
#define HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Social {
        class RobotInterface;
    }
}

class hriPhysio::Social::RobotInterface {

protected:
    std::string name;

public:
    RobotInterface();

    ~RobotInterface();

    void setName(const std::string name);
    std::string getName() const;
    
    enum peripheral { HEAD, RIGHTARM, LEFTARM, RIGHTLEG, LEFTLEG };
    
    virtual bool setPerphState(const peripheral perph, const std::vector<double>& pos) {
        warning("setPerphState");
    }

    virtual bool getPerphState(const peripheral perph, std::vector<double>& pos) {
        warning("getPerphState");
    }

    enum expression { }

    virtual 





    virtual bool openInputStream() = 0;
    virtual bool openOutputStream() = 0;

    virtual void publish(const std::vector<hriPhysio::varType>&  buff, const std::vector<double>* timestamps = nullptr) = 0;
    virtual void receive(std::vector<hriPhysio::varType>& buff, std::vector<double>* timestamps = nullptr) = 0;

private:
    void warning(std::string func) {
        std::cerr << "[DEBUG] " << "Function ``" << func << "`` has not been implemented!!" << std::endl;
    }

};

#endif /* HRI_PHYSIO_SOCIAL_ROBOT_INTERFACE_H */
