/* ================================================================================
 * Copyright: (C) 2020, SIRRL Social and Intelligent Robotics Research Laboratory, 
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

// #include <getopt.h>
// #include <unistd.h>
// #include <signal.h>
// #include <sys/wait.h>

#include <HriPhysio/Manager/physioManager.h>
#include <HriPhysio/Stream/streamerInterface.h>
#include <HriPhysio/Factory/streamerFactory.h>
#include <HriPhysio/helpers.h>

int main (int argc, char **argv) {

    hriPhysio::InputParser input(argc, argv);

    const std::string &filename = input.getCmdOption("--file");

    std::cout << "filename: " << filename << std::endl;


    hriPhysio::Factory::StreamerFactory factory();
    hriPhysio::Stream::StreamerInterface* streamer;

    streamer = factory.getStreamer("string");

    

    return 0;
}