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

#ifndef HRI_PHYSIO_QT_CONTROLLER_H
#define HRI_PHYSIO_QT_CONTROLLER_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <HriPhysio/Social/robotInterface.h>
#include <HriPhysio/helpers.h>

class QtController : public hriPhysio::Social::RobotInterface {

private:
    
    ros::Publisher speech_say_pub;
    ros::Publisher head_pos_pub;
    ros::Publisher right_arm_pos_pub;
    ros::Publisher left_arm_pos_pub;

public:
    QtController();

    ~QtController();

    bool configure(int argc, char **argv);

    void setName(const std::string name);
    std::string getName() const;
    
    bool setPerphState(const peripheral perph, const std::vector<double>& pos);

    bool getPerphState(const peripheral perph, std::vector<double>& pos);

    bool setPerphVelocity(const peripheral perph, const std::vector<double>& speed);

    bool getPerphVelocity(const peripheral perph, std::vector<double>& speed);

    bool setEmotionState(const std::string emotion);

    bool getEmotionState(std::string& emotion);

    bool addSpeech(const std::string phrase);

    bool addAudioFile(const std::string filename, const size_t channel=-1);

private:
    void temp();

};

#endif /* HRI_PHYSIO_QT_CONTROLLER_H */
