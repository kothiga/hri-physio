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

#ifndef HRI_PHYSIO_QT_PHYSIO_COACH_H
#define HRI_PHYSIO_QT_PHYSIO_COACH_H

#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <HriPhysio/Core/ringBuffer.h>
#include <HriPhysio/Manager/threadManager.h>
#include <HriPhysio/helpers.h>

class QtPhysioCoach : public hriPhysio::Manager::ThreadManager {

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
    ros::Subscriber heart_rate_sub;
    hriPhysio::Core::RingBuffer<double> inbox;
    

public:
    QtPhysioCoach();

    ~QtPhysioCoach();

    bool configure(int argc, char **argv);

    void interactive();


private:
    bool threadInit();

    

    void inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

};

#endif /* HRI_PHYSIO_QT_PHYSIO_COACH_H */
