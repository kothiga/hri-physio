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

    //-- Init an argument parser.
    hriPhysio::ArgParser args(argc, argv);

    //-- Get the config file.
    const std::string &yaml_file = args.getCmdOption("--conf");
    YAML::Node config;
    
    try {
        config = YAML::LoadFile(yaml_file);
    } catch (YAML::BadFile&) {
        std::cerr << "[ERROR] "
                  << "Could not load configuration file ``"
                  << yaml_file << "``!! Loading default arguments . . ." 
                  << std::endl;
        config = YAML::Load("");
    }

    // default string vector.
    const std::vector<std::string> empty = {};


    //-- Parameters for the participant.
    part_name = config["part_name"].as<std::string>( /* default=*/ "JohnDoe" );
    part_age  = config["part_age" ].as<double>(      /* default=*/  28       );


    //-- Parameters for ROS streams.
    const std::string input_name  = config["input" ].as<std::string>( /*default=*/ "/input"  );
    const std::string output_name = config["output"].as<std::string>( /*default=*/ "/output" );


    //-- Parameters for the audio and video path.
    audio_path = config["audio_path"].as<std::string>( /*default=*/ "./audio/{}.wav" );
    video_path = config["video_path"].as<std::string>( /*default=*/ "./video/{}.mp4" );


    //-- Parameters for audio files.
    audio_default        = config["audio_default"       ].as<std::string>( /*default=*/ "default"      );
    audio_relaxing       = config["audio_relaxing"      ].as<std::string>( /*default=*/ "relaxing"     );
    audio_exercise_base  = config["audio_exercise_base" ].as<std::string>( /*default=*/ "Exercise_{}x" );
    audio_exercise_tempo = config["audio_exercise_tempo"].as<std::vector<std::string> >( /*default=*/ empty );


    //-- Parameters for video files.
    video_default  = config["video_default" ].as<std::string>( /*default=*/ "default" );
    video_relaxing = config["video_relaxing"].as<std::string>( /*default=*/ "relaxing" );
    video_prefix   = config["video_prefix"  ].as<std::string>( /*default=*/ "{}" );


    //-- Parameters for gestures.
    gesture_default  = config["gesture_default" ].as<std::string>( /*default=*/ "default" );
    gesture_relaxing = config["gesture_relaxing"].as<std::string>( /*default=*/ "relaxing" );
    gesture_prefix   = config["gesture_prefix"  ].as<std::string>( /*default=*/ "{}" );


    //-- Fill for the gesture and video prefix.
    exercises = config["exercises"].as<std::vector<std::string> >( /*default=*/ empty );


    //-- Lists of phrases for Qt to randomly pull from.
    speech_relaxation  = config["speech_relaxation" ].as<std::vector<std::string> >( /*default=*/ empty );
    speech_motivation  = config["speech_motivation" ].as<std::vector<std::string> >( /*default=*/ empty );
    emotion_motivation = config["emotion_motivation"].as<std::vector<std::string> >( /*default=*/ empty );

    calib_time = config["calib_time"].as<double>( /*default=*/ 180.0 );
    calib_skip = config["calib_skip"].as<bool>(   /*default=*/ false );

    //-- Set resting heart rate if skipping calibration.
    if (calib_skip) {
        HRresting = config["resting_default"].as<double>( /*default=*/ 61.0 );
    }


    //-- Other system parameters.
    buffer_length = config["buffer"].as<size_t>( /*default=*/ 1000);


    //-- Do some error checking.
    if (exercises.size() < 4) {
        std::cerr << "[Error] "
                  << "Not enough exercises provided!!" 
                  << std::endl;
        return false;
    }


    //-- Open the ROS streams.
    ros::NodeHandle nh;
    heart_rate_sub    = nh.subscribe(input_name, 1000, &QtPhysioCoach::inputCallback, this);
    qt_controller_pub = nh.advertise<std_msgs::String>(output_name, 1000);
    

    //-- Set the length of the ring buffer.
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
    std::vector<std::string> input = hriPhysio::parseString(inp);
    
    //-- If line was empty, skip to next input.
    if (input.size() == 0) { return; }
    
    //-- Get the command.
    std::string cmd = input[0];
    hriPhysio::toLower(cmd);
    if (cmd == "exit") {

        //-- Close the threads.
        this->close();

    } else if (cmd == "start") {

        //-- Start all threads.
        this->start();
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

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is active, run.
        if (this->getThreadStatus(thread_id)) {

            this->calibrate();
            

            this->sendMessage("exit");

            std::this_thread::sleep_for(
                std::chrono::duration<double>( 5.0 ) //seconds.
            );

            this->close();

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

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();

    //-- Get the current time at the start.
    auto start = std::chrono::system_clock::now();

    //-- Start the relaxing audio and video.
    std::string str;
    str = fmt::format("set audio {}", fmt::format(audio_path, audio_relaxing));
    this->sendMessage(str);

    str = fmt::format("set video {}", fmt::format(video_path, video_relaxing));
    this->sendMessage(str);

    if (!this->calib_skip) {

        //-- Give some instructional speech.
        str = fmt::format("set speech {}", fmt::format(
            "Hello {}, my name is QT, and I will be your "
            "personal trainer for today's session. "
            "I've been told that you are {} years old. "
            "For the next {} minutes, I am going to "
            "read your heart rate. This will give me a good "
            "idea as to what your resting heart rate is. "
            "From there, I can predict what your maximal heart rate is. "
            "This will allow me to better choose a range to keep you active in! "
            "For now, just sit back, listen to this nice music, and take big breaths. "
            "Let's get started!"
            , this->part_name, this->part_age, (int)(this->calib_time/60)
        ));
        this->sendMessage(str);

        //-- Set a time for when the last ``event`` occurred.
        auto event = std::chrono::system_clock::now();
        auto clock = std::chrono::system_clock::now();

        while (true) {
            
            //-- Get the time and see if we're done calibrating.
            auto current = std::chrono::system_clock::now();
            std::chrono::duration<double> dur = current - start;
            if (dur.count() > calib_time || !this->getThreadStatus(thread_id)) {
                break;
            }

            
            //-- Give some random speech or emotion.
            dur = current - event;
            if (dur.count() > 30.0) {

                //-- Reset time since last event.
                event = std::chrono::system_clock::now();

                //-- Choose an emotion to display 75% of the time.
                size_t what = rand() % 4;
                if (what) { //1, 2, 3
                    str = fmt::format("set emotion {}", 
                        hriPhysio::chooseRandom(this->emotion_motivation)
                    );
                } else { // 0
                    str = fmt::format("set speech {}",
                        hriPhysio::chooseRandom(this->speech_relaxation)
                    );
                }
                this->sendMessage(str);
            }


            //-- Tell the user how much time is left.
            dur = current - clock;
            if (dur.count() > 60.0) {

                //-- Reset time since last clock check.
                clock = std::chrono::system_clock::now();

                dur = current - start;
                size_t time_left = (int)(this->calib_time/60) - (int)(dur.count()/60);
                std::cerr << "[DEBUG] " 
                          << "time_left = " << time_left
                          << " = " << (int)(this->calib_time/60) 
                          << " - " << (int)(dur.count()/60)
                          << std::endl;

                str = fmt::format("set speech {}", fmt::format(
                    "You're doing great {}!! "
                    "Only {} minute{} left."
                    , this->part_name, time_left, ((time_left > 1) ? "s" : "")
                ));
                this->sendMessage(str);

            }


            //-- Sleep for some time.
            //-- Wait some time for Qt to complete the above speech.
            std::this_thread::sleep_for(
                std::chrono::duration<double>( 1.0 ) //seconds
            );
        }

        lock.lock();
        std::vector<double> HRbuffer(inbox.size());
        inbox.dequeue(HRbuffer.data(), inbox.size());
        lock.unlock();

        HRresting = hriPhysio::mean(HRbuffer);
    }
    
    // Tanaka, H., Monahan, K. D., & Seals, D. R. (2001). 
    // Age-predicted maximal heart rate revisited. 
    // Journal of the american college of cardiology, 
    // 37(1), 153-156.
    HRmax = HeartRateConst - (AgeConst * part_age);
    HRR   = HRmax - HRresting;

    //-- Compute the 40% and 70% of participants HRR.
    HRR_40 = (0.4 * HRR) + HRresting;
    HRR_70 = (0.7 * HRR) + HRresting;

    //-- Set the video back to being the splash.
    str = fmt::format("set video {}", fmt::format(video_path, video_default));
    this->sendMessage(str);

    //-- Congratulate user. Tell them between 40 and 70
    str = fmt::format("set speech {}", fmt::format(
        "My calibrations are complete!! Thank you for your patience. "
        "I measured your resting heart rate to be {} beats per minute. "
        "With this, I am going to do my best to get your "
        "heart rate between {} and {} beats per minute."
        , HRresting, HRR_40, HRR_70
    ));
    this->sendMessage(str);

    //-- Wait some time for Qt to complete the above speech.
    std::this_thread::sleep_for(
        std::chrono::duration<double>( 10.0 ) //seconds
    );

    return;
}


void QtPhysioCoach::sendMessage(const std::string& message) {
    
    std::cerr << "[SENDING] " << message << std::endl;

    std_msgs::String msg;
    msg.data = message;
    qt_controller_pub.publish(msg); 

    std::this_thread::sleep_for(
        std::chrono::duration<double>( 0.1 ) //seconds
    );

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
