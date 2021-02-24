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

#include <HriPhysio/Manager/robotManager.h>

using namespace hriPhysio::Manager;


RobotManager::RobotManager(hriPhysio::Social::RobotInterface* robot) : 
    robot(robot) {
    
}


RobotManager::~RobotManager() {
    delete robot;
}


void RobotManager::configure(int argc, char **argv) {

    
    robot->configure(argc, argv);





    //-- Load the yaml file.
    //YAML::Node config = YAML::LoadFile(yaml_file);

    //-- Parameters about the streamer.
    //const std::string input_name  = config["input"].as<std::string>();
    //const std::string output_name = config["output"].as<std::string>();

    //-- Parameters about the data.
    //dtype          = config[ "dtype"          ].as<std::string>( /*default=*/ "int32");
    //sampling_rate  = config[ "sampling_rate"  ].as<std::size_t>( /*default=*/ 20  );
    //input_frame    = config[ "input_frame"    ].as<std::size_t>( /*default=*/ 10  );
    //output_frame   = config[ "output_frame"   ].as<std::size_t>( /*default=*/ 20  );
    //num_channels   = config[ "num_channels"   ].as<std::size_t>( /*default=*/ 1   );
    //sample_overlap = config[ "sample_overlap" ].as<std::size_t>( /*default=*/ 0   );
    //buffer_length  = config[ "buffer_length"  ].as<std::size_t>( /*default=*/ 100 );

    //-- Enable logging?
    //log_data = config["log_data"].as<bool>( /*default=*/ false );
    //log_name = config["log_name"].as<std::string>( /*default=*/ "");

    std::cerr << "[CONF] Load complete.\n";
    

    //-- If streams are empty, exit.
    //if (input_name == "" || output_name == "") {
    //    this->close();
    //    return;
    //}


    //-- Configure the streams.
    //stream_input->setName(input_name);
    //stream_input->setDataType(dtype);
    //stream_input->setFrameLength(input_frame);
    //stream_input->setNumChannels(num_channels);
    //stream_input->setSamplingRate(sampling_rate);
    //
    //stream_output->setName(output_name);
    //stream_output->setDataType(dtype);
    //stream_output->setFrameLength(output_frame);
    //stream_output->setNumChannels(num_channels);
    //stream_output->setSamplingRate(sampling_rate);

    //if (log_data) {
    //    stream_logger.setName(log_name);
    //    stream_logger.setDataType(dtype);
    //    stream_logger.setFrameLength(input_frame);
    //    stream_logger.setNumChannels(num_channels);
    //    stream_logger.setSamplingRate(sampling_rate);
    //}


    //-- Try opening the streams.
    //if (!stream_input->openInputStream()) {
    //    std::cerr << "Could not open input stream.\n";
    //    this->close();
    //    return;
    //}
    //
    //if (!stream_output->openOutputStream()) {
    //    std::cerr << "Could not open output stream.\n";
    //    this->close();
    //    return;
    //}
    //
    //if (log_data && !stream_logger.openOutputStream()) {
    //    std::cerr << "Could not open logger stream.\n";
    //    this->close();
    //    return;
    //}


    //-- Initialize the threads.
    this->threadInit();

    return;
}


void RobotManager::interactive() {
    std::string cmd, inp, str;
    while (this->getManagerRunning()) {
        
        //-- Get some input.
        std::cout << ">>> ";
        getline(std::cin, inp);

        //-- Parse it up.
        std::vector< std::string > input = hriPhysio::parseString(inp);
        
        //-- If line was empty, skip to next input.
        if (input.size() == 0) { continue; }
        
        //-- Get the command.
        cmd = input[0];
        hriPhysio::toLower(cmd);
        if (cmd == "exit") {

            //-- Close the threads.
            this->close();
            
            break;

        } else if (cmd == "set") {

            if(!setFunctions(input)) {
                std::cerr << "[ERROR] "
                          << "Unrecognized set input ``"
                          << hriPhysio::combineString(input, 1)
                          << "``" << std::endl;
            }

        } else if (cmd == "get") {

            if(!getFunctions(input)) {
                std::cerr << "[ERROR] "
                          << "Unrecognized get input ``"
                          << hriPhysio::combineString(input, 1)
                          << "``" << std::endl;
            }


        } else if (cmd == "help") {
            
        } else {
            std::cerr << "[ERROR] "
                      << "Unrecognized command ``"
                      << cmd << "``" << std::endl;
        }
    }
}


bool RobotManager::threadInit() {

    //-- Initialize threads but don't start them yet.
    //addThread(std::bind(&PhysioManager::inputLoop, this),  /*start=*/ false);
    //addThread(std::bind(&PhysioManager::outputLoop, this), /*start=*/ false);

    return true;
}


bool RobotManager::setFunctions(const std::vector< std::string >& input) {

    std::string func = input[1];
    hriPhysio::toLower(func);

    if (func == "state") {
        // set state head 0.0 0.0
        // right-arm 0.0 0.0 0.0
        // left-arm 0.0 0.0 0.0

        std::string perph = input[2];
        hriPhysio::toLower(perph);

        std::vector< double > pos = hriPhysio::toVecDouble(input, 3);

        if (perph == "head") { //TODO: check input range and length.
            robot->setPerphState(hriPhysio::Social::RobotInterface::peripheral::HEAD, pos);
        }

        //setPerphState(const peripheral perph, const std::vector<double>& pos);
    
    } else if (func == "velocity") {
        //setPerphVelocity(const peripheral perph, const std::vector<double>& speed);

    } else if (func == "emotion") {
        //setEmotionState(const std::string emotion);

    } else if (func == "speech") {

        //-- Combine the string and send it out.
        std::string phrase = hriPhysio::combineString(input, 2);
        return robot->addSpeech(phrase);

    } else if (func == "audio") {
        //addAudioFile(const std::string filename, const size_t channel=-1);

    } else {
        return false;
    }

    return true;
}


bool RobotManager::getFunctions(const std::vector< std::string >& input) {

    std::string func = input[1];
    hriPhysio::toLower(func);

    if (func == "state") {
        //getPerphState(const peripheral perph, std::vector<double>& pos);
    
    } else if (func == "velocity") {
        //getPerphVelocity(const peripheral perph, std::vector<double>& speed);

    } else if (func == "emotion") {
        //getEmotionState(std::string& emotion);

    } else {
        return false;
    }

    return true;
}


//void RobotManager::inputLoop() {
//
//    //-- Get the id for the current thread.
//    const std::thread::id thread_id = std::this_thread::get_id();
//
//    //-- Construct a vector for moving data between stream and the buffer.
//    std::vector<hriPhysio::varType> transfer(input_frame * num_channels);
//    std::vector<double> stamps(input_frame);
//
//    //-- Loop until the manager stops running.
//    while (this->getManagerRunning()) {
//        
//        //-- If this thread is active, run.
//        if (this->getThreadStatus(thread_id)) {
//
//            //-- Get data from the stream.
//            stream_input->receive(transfer, &stamps);
//
//            if (transfer.size()) {
//
//                //for (std::size_t idx = 0; idx < transfer.size(); idx++) {
//                //    std::cerr << " [" << stamps[idx] << "]: "; std::visit(hriPhysio::printVisitor(), transfer[idx]);
//                //} std::cerr << std::endl;
//
//                //-- Add the data to the buffer.
//                this->buffer.enqueue(transfer.data(), transfer.size());
//                this->timestamps.enqueue(stamps.data(), stamps.size());
//
//
//                if (this->log_data) {
//                    std::cerr << "Logging.\n";
//                    stream_logger.publish(transfer, &stamps);
//                }
//            }
//
//
//        } else {
//            std::this_thread::sleep_for(
//                std::chrono::duration<double>( 0.1 ) //seconds.
//            );
//        }
//    }
//
//}
