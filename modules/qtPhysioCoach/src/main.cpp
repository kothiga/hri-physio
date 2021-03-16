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

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <qtPhysioCoach.h>

#include <HriPhysio/helpers.h>

int main (int argc, char **argv) {

    //-- Init the ros node.
    ros::init(argc, argv, "QtPhysioCoach", ros::init_options::AnonymousName);
    

    //-- Init an argument parser.
    hriPhysio::ArgParser args(argc, argv);


    //-- Get some vars from command line.
    const bool interactive_mode = args.cmdOptionExists("--interactive");


    //-- Create a robot interface.



    //-- Shut down nodes.
    ros::shutdown();
    
    return 0;
}

