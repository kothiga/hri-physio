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

#include <qtController.h>

#include <HriPhysio/Manager/robotManager.h>
#include <HriPhysio/Social/robotInterface.h>
#include <HriPhysio/helpers.h>

int main (int argc, char **argv) {

    //-- Init the ros node.
    ros::init(argc, argv, "QtController", ros::init_options::AnonymousName);
    

    //-- Init an argument parser.
    hriPhysio::ArgParser args(argc, argv);


    //-- Get some vars from command line.
    const bool interactive_mode = args.cmdOptionExists("--interactive");


    //-- Create a robot interface.
    hriPhysio::Social::RobotInterface* robot = new QtController();


    //-- Initialize the manager, and pass it the config file.
    hriPhysio::Manager::RobotManager manager(robot);
    manager.configure(argc, argv);


    //-- Start the manager.
    manager.start();


    //-- Run in interactive mode if enabled.
    if (interactive_mode) {
        manager.interactive();
    }

    
    //-- Wait for everything to finish.
    manager.wait();


    //-- Shut down nodes.
    ros::shutdown();
    
    return 0;
}
