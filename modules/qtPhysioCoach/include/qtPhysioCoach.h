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
#include <yaml-cpp/yaml.h>

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
    double      part_age;

    size_t buffer_length;
    size_t speed_idx;
    std::vector< std::string > speed_modifier;
    

    double calib_time;
    bool   calib_skip;

    std::string audio_path;
    std::string video_path;

    std::string audio_default;
    std::string audio_relaxing;
    std::string audio_exercise_base;
    std::vector< std::string > audio_suffix;

    std::string video_default;
    std::string video_relaxing;
    std::string video_prefix;
    double      video_time;

    std::string gesture_default;
    std::string gesture_relaxing;
    std::string gesture_prefix;

    std::vector< std::string > exercises;

    std::vector< std::string > speech_relaxation;
    std::vector< std::string > speech_motivation;
    std::vector< std::string > speech_faster;
    std::vector< std::string > speech_slower;
    std::vector< std::string > emotion_motivation;


    //-- Variables derived from calibration phase.
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

    void runExercise(const std::string typeExercise, const double seconds);

    void sendMessage(const std::string& message, const double sleep_post=0.0);

    void inputLoop();

    void inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

};

#endif /* HRI_PHYSIO_QT_PHYSIO_COACH_H */
