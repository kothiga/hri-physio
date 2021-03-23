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

    ros::NodeHandle nh;
    
    //-- Open Moter Interfaces.
    head_pos_pub      = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/head_position/command", 10);
    right_arm_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/right_arm_position/command", 10);
    left_arm_pos_pub  = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/left_arm_position/command", 10);

    //-- Open Behavior Interfaces.
    emotion_show_pub    = nh.advertise<std_msgs::String>("/qt_robot/emotion/show", 10);
    gesture_play_client = nh.serviceClient<qt_gesture_controller::gesture_play>("/qt_robot/gesture/play");
    
    //-- Open Speech Interface.
    //speech_say_pub      = nh.advertise<std_msgs::String>("/qt_robot/speech/say", 10); // something to try out...
    speech_say_pub       = nh.advertise<std_msgs::String>("/qt_robot/behavior/talkText", 10); 
    speech_config_client = nh.serviceClient<qt_robot_interface::speech_config>("/qt_robot/speech/config");
    set_volume_client    = nh.serviceClient<qt_robot_interface::setting_setVolume>("/qt_robot/setting/setVolume");

    //-- Open Other Interfaces.
    audio_file_pub = nh.advertise<std_msgs::String>("/audio_streamer/audio_name", 10);
    video_file_pub = nh.advertise<std_msgs::String>("/video_streamer/video_name", 10);

    //-- Open the input stream.
    command_sub = nh.subscribe("/QtController/input", 1000, &QtController::inputCallback, this);

    return true;
}


void QtController::robotLoop() {
    ros::spinOnce();
    return;   
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
        
        this->head_pos_pub.publish(msg);
        break;

    case peripheral::RIGHTARM:

        //-- Check that given 3 joint positions.
        if (pos.size() != 3) { return false; }
        ROS_INFO("[QT-state] rightarm %f %f %f", pos[0], pos[1], pos[2]);
        
        this->right_arm_pos_pub.publish(msg);
        break;
    
    case peripheral::LEFTARM:

        //-- Check that given 3 joint positions.
        if (pos.size() != 3) { return false; }
        ROS_INFO("[QT-state] leftarm %f %f %f", pos[0], pos[1], pos[2]);

        this->left_arm_pos_pub.publish(msg);
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
    
    this->emotion_show_pub.publish(msg);

    return true;
}


bool QtController::getEmotionState(std::string& emotion) {
    

    return true;
}


bool QtController::addGesture(const std::string gesture, const double speed/*=1*/) {

    ROS_INFO("[QT-gesture] ``%s`` %f", gesture.c_str(), speed);

    qt_gesture_controller::gesture_play cmd;

    cmd.request.name  = gesture;
    cmd.request.speed = speed;

    if(!this->gesture_play_client.call(cmd)) {
        ROS_WARN("[QT-gesture] Could not call service gesture_play");
    }

    return cmd.response.status;
}


bool QtController::addSpeech(const std::string phrase) {
    
    ROS_INFO("[QT-speech] ``%s``", phrase.c_str());

    std_msgs::String msg;
    msg.data = phrase;
    this->speech_say_pub.publish(msg);

    return true;
}


bool QtController::setSpeechConfig(const std::string config) {

    ROS_INFO("[QT-speech-config] ``%s``", config.c_str());

    std::vector< std::string > vec = hriPhysio::parseString(config);

    if (vec.size() != 2) { 
        ROS_WARN("QT-speech-config] Requires 2 arguments... given %ld", vec.size());
        return false; 
    }

    //the default pitch is usually '140' and speed is '80'.

    qt_robot_interface::speech_config cmd;

    cmd.request.pitch = std::stoi(vec[0]);
    cmd.request.speed = std::stoi(vec[1]);
    
    if(!this->speech_config_client.call(cmd)) {
        ROS_WARN("[QT-speech-config] Could not call service speech_config");
    }

    return cmd.response.status;
}


bool QtController::setVolume(const double percent) {

    ROS_INFO("[QT-volume] %f", percent);

    qt_robot_interface::setting_setVolume cmd;

    cmd.request.volume = percent; // (double) -> (int)

    if(!this->set_volume_client.call(cmd)) {
        ROS_WARN("[QT-volume] Could not call service setting_setVolume");
    }

    return cmd.response.status;
}


bool QtController::addAudioFile(const std::string filename, const size_t channel/*=-1*/) {
    
    ROS_INFO("[QT-audio] ``%s``", filename.c_str());

    std_msgs::String msg;
    msg.data = filename;
    this->audio_file_pub.publish(msg);
    
    return true;
}


bool QtController::addVideoFile(const std::string filename) {
    
    ROS_INFO("[QT-video] ``%s``", filename.c_str());

    std_msgs::String msg;
    msg.data = filename;
    this->video_file_pub.publish(msg);
    
    return true;
}


bool QtController::getRobotCommand(std::string& command) {

    if (inbox.empty()) { return false; }

    lock.lock();
    command = inbox.front(); 
    inbox.pop();
    lock.unlock();

    return true;
}


void QtController::inputCallback(const std_msgs::String::ConstPtr& msg) {
    
    std::string str = msg->data.c_str();
    //ROS_INFO("I heard: [%s]", str.c_str());

    lock.lock();
    inbox.push( str );
    lock.unlock();

    return;
}
