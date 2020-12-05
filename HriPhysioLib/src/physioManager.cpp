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

#include <HriPhysio/Manager/physioManager.h>

using namespace hriPhysio::Manager;


PhysioManager::PhysioManager(hriPhysio::Stream::StreamerInterface* input, hriPhysio::Stream::StreamerInterface* output) : 
    stream_input(input),
    stream_output(output) {
    
}


PhysioManager::~PhysioManager() {
    delete stream_input;
    delete stream_output;
}


void PhysioManager::configure(const std::string yaml_file) {

    //-- Load the yaml file.
    YAML::Node config = YAML::LoadFile(yaml_file);

    //-- Parameters about the streamer.
    const std::string input_name  = config["input"].as<std::string>();
    const std::string output_name = config["output"].as<std::string>();

    //-- Parameters about the data.
    dtype          = config[ "dtype"          ].as<std::string>(/*default=*/ "int32");
    sampling_rate  = config[ "sampling_rate"  ].as<std::size_t>( /*default=*/ 20  );
    input_frame    = config[ "input_frame"    ].as<std::size_t>( /*default=*/ 10  );
    output_frame   = config[ "output_frame"   ].as<std::size_t>( /*default=*/ 20  );
    num_channels   = config[ "num_channels"   ].as<std::size_t>( /*default=*/ 1   );
    sample_overlap = config[ "sample_overlap" ].as<std::size_t>( /*default=*/ 0   );
    buffer_length  = config[ "buffer_length"  ].as<std::size_t>( /*default=*/ 100 );

    //-- Enable logging?
    //const bool log_data = config["log"].as<bool>();
    

    //-- Configure the intermediary buffer.
    buffer.resize(buffer_length * num_channels);

    //-- If streams are empty, exit.
    if (input_name == "" || output_name == "") {
        this->close();
        return;
    }


    //-- Configure the streams.
    stream_input->setName(input_name);
    stream_input->setDataType(dtype);
    stream_input->setFrameLength(input_frame);
    stream_input->setNumChannels(num_channels);

    stream_output->setName(output_name);
    stream_output->setDataType(dtype);
    stream_output->setFrameLength(output_frame);
    stream_output->setNumChannels(num_channels);


    //-- Try opening the streams.
    if (!stream_input->openInputStream()) {
        std::cerr << "Could not open input stream";
        this->close();
        return;
    }

    if (!stream_output->openOutputStream()) {
        std::cerr << "";
        this->close();
        return;
    }


    //-- Initialize the threads.
    this->threadInit();

    return;
}


void PhysioManager::interactive() {
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


void PhysioManager::wait() {

    //-- Wait for the thread manager to stop.
    while (this->getManagerRunning()) {
        std::this_thread::sleep_for(
            std::chrono::duration<double>( 0.5 ) //seconds.
        );
    }

    return;
}


bool PhysioManager::threadInit() {

    //-- Initialize threads but don't start them yet.
    addThread(std::bind(&PhysioManager::inputLoop, this),  /*start=*/ false);
    addThread(std::bind(&PhysioManager::outputLoop, this), /*start=*/ false);

    return true;
}


void PhysioManager::inputLoop() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Construct a vector for moving data between stream and the buffer.
    std::vector<hriPhysio::varType> transfer; //TODO: allocate space.

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is 
        if (this->getThreadStatus(thread_id)) {

            //-- Get data from the stream.
            stream_input->receive(transfer, num_channels);

            //-- Add the data to the buffer.
            buffer.enqueue(transfer.data(), transfer.size());

            //TODO: log.

        } else {
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 0.1 ) //seconds.
            );
        }
    }

}


void PhysioManager::outputLoop() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Construct a vector for moving data between the buffer and stream.
    std::vector<hriPhysio::varType> transfer;
    transfer.reserve(output_frame);

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is 
        if (this->getThreadStatus(thread_id) && buffer.size() >= output_frame) {

            buffer.dequeue(transfer.data(), output_frame, sample_overlap);

            stream_output->publish(transfer, num_channels);

        } else {
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 0.01 ) //seconds.
            );
        }
    }
}
