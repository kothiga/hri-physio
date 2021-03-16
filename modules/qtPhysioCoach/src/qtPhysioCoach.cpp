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

#include <qtPhysioCoach.h>


QtPhysioCoach::QtPhysioCoach() {

}


QtPhysioCoach::~QtPhysioCoach() {
}



bool QtPhysioCoach::configure(int argc, char **argv) {

    return true;
}


void QtPhysioCoach::interactive() {
    std::string str;
    while (this->getManagerRunning()) {
        std::cin >> str;
        std::cout << ">>> " << str << std::endl;

        if (str == "exit") {

            //-- Close the threads.
            this->close();
            
            break;
        }
    }
}


bool QtPhysioCoach::threadInit() {

    //-- Initialize threads but don't start them yet.
    //addThread(std::bind(&PhysioManager::inputLoop, this),  /*start=*/ false);
    //addThread(std::bind(&PhysioManager::outputLoop, this), /*start=*/ false);

    return true;
}


void QtPhysioCoach::inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    
    ROS_INFO("Got Vec with len %ld", msg->data.size());

    inbox.enqueue(msg->data.data(), msg->data.size());

    return;
}
