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
    
    //-- Controller interface.
    ros::Publisher qt_controller_pub;

    //-- Input stream.
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
