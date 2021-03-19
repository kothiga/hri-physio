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

    ros::NodeHandle nh;

    //-- TODO: Replace these with strings from conf.
    qt_controller_pub = nh.advertise<std_msgs::String>("/QtController/input", 1000);
    heart_rate_sub    = nh.subscribe("/PhysioStreamer/hr", 1000, &QtPhysioCoach::inputCallback, this);

    std::cerr << "Conf complete." << std::endl;






    inbox.resize(buffer_length);




    //-- Initialize the threads.
    this->threadInit();

    return true;
}


void QtPhysioCoach::interactive() {
     
    std::string inp;
    while (this->getManagerRunning()) {
        
        //-- Get some input.
        std::cout << ">>> ";
        getline(std::cin, inp);

        //-- Process the request.
        this->process(inp);
    }    
}


void QtPhysioCoach::process(const std::string& inp) {

    //-- Parse it up.
    std::vector< std::string > input = hriPhysio::parseString(inp);
    
    //-- If line was empty, skip to next input.
    if (input.size() == 0) { return; }
    
    //-- Get the command.
    std::string cmd = input[0];
    hriPhysio::toLower(cmd);
    if (cmd == "exit") {

        //-- Close the threads.
        this->close();

    }
}


bool QtPhysioCoach::threadInit() {

    //-- Initialize threads but don't start them yet.
    addThread(std::bind(&QtPhysioCoach::run, this),  /*start=*/ false);
    addLoopThread(std::bind(&QtPhysioCoach::inputLoop, this), /*period=*/ 0.1, /*start=*/ false);

    return true;
}


void QtPhysioCoach::run() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    std_msgs::String msg;

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is active, run.
        if (this->getThreadStatus(thread_id)) {

            msg.data.clear();
            msg.data = "set emotion QT/happy";
            qt_controller_pub.publish(msg); 


            msg.data.clear();
            msg.data = "set audio /home/austin/exercise/audio/FutureHouse_1.00x.wav";
            qt_controller_pub.publish(msg); 

            msg.data.clear();
            msg.data = "set speech Testing the loop";
            qt_controller_pub.publish(msg); 

            msg.data.clear();
            msg.data = "set gesture exercise/marching 1.0";
            qt_controller_pub.publish(msg); 


            msg.data.clear();
            msg.data = "set audio /home/austin/exercise/audio/FutureHouse_1.15x.wav";
            qt_controller_pub.publish(msg); 

            
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 10.0 ) //seconds.
            );

        } else {
            std::cerr << "sleep . . ." << std::endl;
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 0.1 ) //seconds.
            );
        }
    }

//    if (this->run_calibration) {
//        this->calibrate();
//    }
//
//    for (size_t idx = 0; idx < this->num_exercise; ++idx) {
//        //Frequency
//        //Intensity
//        //Time
//        //Type
//        this->exerciseRoutine()
//    }

    return;
}


void QtPhysioCoach::calibrate() {

    //-- Get the current time at the start.
    auto start = std::chrono::system_clock::now();

    //-- Start the relaxing audio and video.
    std::string str;
    str = fmt::format("set audio {}", fmt::format(audio_pattern, audio_relaxing));
    this->sendMessage(str);

    str = fmt::format("set video {}", fmt::format(video_pattern, video_relaxing));
    this->sendMessage(str);


    //-- Give some instructional speech.


    while (true) {

        auto current = std::chrono::system_clock::now();
        std::chrono::duration<double> dur = current - start;
        if (dur.count() > calib_time) {
            break;
        }

        //-- Give some random speech.


        //-- Sleep for some time.
    }

    lock.lock();
    std::vector<double> HRbuffer(inbox.size());
    inbox.dequeue(HRbuffer.data(), inbox.size());
    lock.unlock();

    HRresting = hriPhysio::mean(HRbuffer);
    
    HRmax = HeartRateConst - (AgeConst * age);
    HRR   = HRmax - HRresting;

    HRR_40 = (0.4 * HRR) + HRresting;
    HRR_70 = (0.7 * HRR) + HRresting;

    // TODO: Congratulate user. Tell them between 40 and 70

    return;
}


void QtPhysioCoach::sendMessage(const std::string& message) {
    
    std::cerr << "[SENDING] " << message << std::endl;

    std_msgs::String msg;
    msg.data = message;
    qt_controller_pub.publish(msg); 

    return;
}


void QtPhysioCoach::inputLoop() {

    ros::spinOnce();

    return;
}

void QtPhysioCoach::inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    
    ROS_INFO("Got Vec with len %ld", msg->data.size());

    lock.lock();
    inbox.enqueue(msg->data.data(), msg->data.size());
    lock.unlock();

    return;
}
