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
    speech_say_pub    = nh.advertise<std_msgs::String>("/qt_robot/speech/say", 10);
    head_pos_pub      = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/head_position/command", 10);
    right_arm_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/right_arm_position/command", 10);
    left_arm_pos_pub  = nh.advertise<std_msgs::Float64MultiArray>("/qt_robot/left_arm_position/command", 10);

    return true;
}


bool QtController::setPerphState(const peripheral perph, const std::vector<double>& pos) {
    
    std_msgs::Float64MultiArray msg;
    for (size_t idx = 0; idx < pos.size(); ++idx) {
        msg.data.push_back( pos[idx] );
    }

    switch (perph) {

    case peripheral::HEAD:
        head_pos_pub.publish(msg);
        break;

    case peripheral::RIGHTARM:
        right_arm_pos_pub.publish(msg);
        break;
    
    case peripheral::LEFTARM:
        left_arm_pos_pub.publish(msg);
        break;
    
    default:
        std::cerr << "[DEBUG] "
                  << "Peripheral input not supported for Qt!!";
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
    

    return true;
}


bool QtController::getEmotionState(std::string& emotion) {
    

    return true;
}


bool QtController::addSpeech(const std::string phrase) {
    
    std::cout << "[QT-OUT] ``" << phrase << "``" << std::endl;

    std_msgs::String msg;
    msg.data = phrase;
    speech_say_pub.publish(msg);

    return true;
}


bool QtController::addAudioFile(const std::string filename, const size_t channel/*=-1*/) {
    

    return true;
}
