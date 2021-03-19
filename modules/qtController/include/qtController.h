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
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <qt_services/gesture_play.h>
#include <qt_services/setting_setVolume.h>
#include <qt_services/speech_config.h>

#include <HriPhysio/Social/robotInterface.h>
#include <HriPhysio/helpers.h>

class QtController : public hriPhysio::Social::RobotInterface {

private:
    
    //-- Motor interface.
    ros::Publisher head_pos_pub;
    ros::Publisher right_arm_pos_pub;
    ros::Publisher left_arm_pos_pub;

    //-- Behavior interface.
    ros::Publisher emotion_show_pub; 
    ros::ServiceClient gesture_play_client;
    
    //-- Speech interface
    ros::Publisher speech_say_pub;
    ros::ServiceClient speech_config_client;
    ros::ServiceClient set_volume_client;
    
    //-- Other interface.
    ros::Publisher audio_file_pub;
    ros::Publisher video_file_pub;

    //-- Input commands.
    ros::Subscriber command_sub;
    std::queue< std::string > inbox;
    
    //-- Atomicity.
    std::mutex lock;

public:
    QtController();

    ~QtController();

    bool configure(int argc, char **argv);

    void robotLoop();
    
    bool setPerphState(const peripheral perph, const std::vector<double>& pos);

    bool getPerphState(const peripheral perph, std::vector<double>& pos);

    bool setPerphVelocity(const peripheral perph, const std::vector<double>& speed);

    bool getPerphVelocity(const peripheral perph, std::vector<double>& speed);

    bool setEmotionState(const std::string emotion);

    bool getEmotionState(std::string& emotion);

    bool addGesture(const std::string gesture, const double speed=1.0);

    bool addSpeech(const std::string phrase);

    bool setSpeechConfig(const std::string config);

    bool setVolume(const double percent);

    bool addAudioFile(const std::string filename, const size_t channel=-1);

    bool addVideoFile(const std::string filename);

    bool getRobotCommand(std::string& command);

private:
    void inputCallback(const std_msgs::String::ConstPtr& msg);

};

#endif /* HRI_PHYSIO_QT_CONTROLLER_H */
