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
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <queue>
#include <vector>

#include <fmt/core.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

#include <HriPhysio/Core/ringBuffer.h>
#include <HriPhysio/Manager/threadManager.h>
#include <HriPhysio/helpers.h>

class QtPhysioCoach : public hriPhysio::Manager::ThreadManager {

private:
    
    //-- Controller interface.
    ros::Publisher qt_controller_pub;

    //-- Input stream.
    ros::Subscriber heart_rate_sub;
    hriPhysio::Core::RingBuffer<double> inbox;

    //-- Mutex for atomicity.
    std::mutex lock;

    //-- Variables from configuration.
    std::string part_name;
    
    std::string audio_pattern;
    std::string audio_exercise;
    std::string audio_relaxing;

    std::string video_pattern;
    std::string video_relaxing;


    double calib_time;
    size_t buffer_length;

    
    double age;
    double HRmax, HRresting, HRR, HRR_40, HRR_70;


    //-- Parameters from:
    // Tanaka, H., Monahan, K. D., & Seals, D. R. (2001). 
    // Age-predicted maximal heart rate revisited. 
    // Journal of the american college of cardiology, 
    // 37(1), 153-156.
    const double HeartRateConst = 208.0;
    const double AgeConst = 0.7;


public:
    QtPhysioCoach();

    ~QtPhysioCoach();

    bool configure(int argc, char **argv);

    void interactive();


private:
    void process(const std::string& inp);

    bool threadInit();

    void run();

    void calibrate();

    void sendMessage(const std::string& message);

    void inputLoop();

    void inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

};

#endif /* HRI_PHYSIO_QT_PHYSIO_COACH_H */
