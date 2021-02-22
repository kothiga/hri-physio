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

#include <HriPhysio/Manager/physioManager.h>
#include <HriPhysio/Stream/streamerInterface.h>
#include <HriPhysio/Factory/streamerFactory.h>
#include <HriPhysio/helpers.h>

int main (int argc, char **argv) {

    //-- Init an argument parser.
    hriPhysio::ArgParser args(argc, argv);


    //-- Get some vars from command line.
    const std::string &yaml_file = args.getCmdOption("--conf");
    const bool interactive_mode  = args.cmdOptionExists("--interactive");

    // TODO: input/output arg type here.
    

    //-- Create some input and output streams.
    hriPhysio::Factory::StreamerFactory   factory;
    hriPhysio::Stream::StreamerInterface* streamer_input;
    hriPhysio::Stream::StreamerInterface* streamer_output;

    // TODO: Make this take YARP or ROS as possible input/output streams.
    //-- Construct the streams. 
    streamer_input  = factory.getStreamer("LSL");
    streamer_output = factory.getStreamer("LSL");


    //-- Initialize the manager, and pass it the config file.
    hriPhysio::Manager::PhysioManager manager(streamer_input, streamer_output);
    manager.configure(yaml_file);


    //-- Start the manager.
    manager.start();

    //-- Run in interactive mode if enabled.
    if (interactive_mode) {
        manager.interactive();
    }

    //-- Wait for everything to finish.
    manager.wait();
    
    return 0;
}
