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

#include <qtController.h>


QtController::QtController() :
    RobotInterface() {

}


QtController::~QtController() {
}


bool QtController::configure(int argc, char **argv) {

    ros::init(argc, argv, "QtController");
    
    ros::NodeHandle nh;
    
    //-- Open Moter Interfaces.
    head_pos_pub      = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/head_position/command", 10);
    right_arm_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/right_arm_position/command", 10);
    left_arm_pos_pub  = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/left_arm_position/command", 10);

    //-- Open Behavior Interfaces.
    emotion_show_pub    = nh.advertise<std_msgs::String>("/qt_robot/emotion/show", 10);
    gesture_play_client = nh.serviceClient<qt_gesture_controller::gesture_play>("/qt_robot/gesture/play");
    speech_say_pub      = nh.advertise<std_msgs::String>("/qt_robot/speech/say", 10);

    //-- Open Other Interfaces.
    audio_file_pub = nh.advertise<std_msgs::String>("", 10);
    video_file_pub = nh.advertise<std_msgs::String>("", 10);    

    return true;
}


bool QtController::setPerphState(const peripheral perph, const std::vector<double>& pos) {

    std_msgs::Float64MultiArray msg;
    for (size_t idx = 0; idx < pos.size(); ++idx) {
        msg.data.push_back( pos[idx] );
    }

    switch (perph) {

    case peripheral::HEAD:
        
        //-- Check that given 2 joint positions.
        if (pos.size() != 2) { return false; }
        ROS_INFO("[QT-state] head %f %f", pos[0], pos[1]);
        
        head_pos_pub.publish(msg);
        break;

    case peripheral::RIGHTARM:

        //-- Check that given 3 joint positions.
        if (pos.size() != 3) { return false; }
        ROS_INFO("[QT-state] rightarm %f %f %f", pos[0], pos[1], pos[2]);
        
        right_arm_pos_pub.publish(msg);
        break;
    
    case peripheral::LEFTARM:

        //-- Check that given 3 joint positions.
        if (pos.size() != 3) { return false; }
        ROS_INFO("[QT-state] leftarm %f %f %f", pos[0], pos[1], pos[2]);

        left_arm_pos_pub.publish(msg);
        break;
    
    default:
        ROS_ERROR("[QT-state] Peripheral input not supported for Qt!!");
        return false;
        break;
    }

    return true;
}


bool QtController::getPerphState(const peripheral perph, std::vector<double>& pos) {
    

    return true;
}


bool QtController::setPerphVelocity(const peripheral perph, const std::vector<double>& speed) {
    
    
    return true;
}


bool QtController::getPerphVelocity(const peripheral perph, std::vector<double>& speed) {
    

    return true;
}


bool QtController::setEmotionState(const std::string emotion) {
    
    ROS_INFO("[QT-emotion] ``%s``", emotion.c_str());

    std_msgs::String msg;
    msg.data = emotion;
    
    emotion_show_pub.publish(msg);

    return true;
}


bool QtController::getEmotionState(std::string& emotion) {
    

    return true;
}


bool QtController::addGesture(const std::string gesture, const double speed/*=1*/) {

    qt_gesture_controller::gesture_play cmd;

    cmd.request.name  = gesture;
    cmd.request.speed = speed;

    if(!gesture_play_client.call(cmd)) {
        ROS_WARN("[QT-gesture] Could not call service gesture_play");
    }

    return cmd.response.status;
}


bool QtController::addSpeech(const std::string phrase) {
    
    ROS_INFO("[QT-speech] ``%s``", phrase.c_str());

    std_msgs::String msg;
    msg.data = phrase;
    speech_say_pub.publish(msg);

    return true;
}


bool QtController::addAudioFile(const std::string filename, const size_t channel/*=-1*/) {
    
    ROS_INFO("[QT-audio] ``%s``", filename.c_str());

    std_msgs::String msg;
    msg.data = filename;
    audio_file_pub.publish(msg);
    
    return true;
}


bool QtController::addVideoFile(const std::string filename) {
    
    ROS_INFO("[QT-video] ``%s``", filename.c_str());

    std_msgs::String msg;
    msg.data = filename;
    video_file_pub.publish(msg);
    
    return true;
}
