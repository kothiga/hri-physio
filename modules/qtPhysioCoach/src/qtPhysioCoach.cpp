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
    
    //-- Try to open the config. If not possible load the defaults.
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


    //-- System variables.
    buffer_length  = config["buffer"   ].as<size_t>( /*default=*/ 1000 );
    speed_idx      = config["speed_idx"].as<size_t>( /*default=*/ 1 );
    speed_modifier = config["speed_modifier"].as<std::vector<std::string>>( /*default=*/ empty );


    //-- Calibration variables.
    calib_time = config["calib_time"].as<double>( /*default=*/ 180.0 );
    calib_skip = config["calib_skip"].as<bool>(   /*default=*/ false );

    //-- Set resting heart rate if skipping calibration.
    if (calib_skip) {
        HRresting = config["resting_default"].as<double>( /*default=*/ 61.0 );
    }


    //-- Parameters for the audio and video path.
    audio_path = config["audio_path"].as<std::string>( /*default=*/ "./audio/{}.wav" );
    video_path = config["video_path"].as<std::string>( /*default=*/ "./video/{}.mp4" );


    //-- Parameters for audio files.
    audio_default       = config["audio_default"      ].as<std::string>( /*default=*/ "default"      );
    audio_relaxing      = config["audio_relaxing"     ].as<std::string>( /*default=*/ "relaxing"     );
    audio_exercise_base = config["audio_exercise_base"].as<std::string>( /*default=*/ "Exercise_{}x" );    
    audio_suffix        = config["audio_suffix"       ].as<std::vector<std::string>>( /*default=*/ empty );


    //-- Parameters for video files.
    video_default  = config["video_default" ].as<std::string>( /*default=*/ "default" );
    video_relaxing = config["video_relaxing"].as<std::string>( /*default=*/ "relaxing" );
    video_prefix   = config["video_prefix"  ].as<std::string>( /*default=*/ "{}" );
    video_time     = config["video_time"    ].as<double>( /*default=*/ 30.0 );


    //-- Parameters for gestures.
    gesture_default  = config["gesture_default" ].as<std::string>( /*default=*/ "default" );
    gesture_relaxing = config["gesture_relaxing"].as<std::string>( /*default=*/ "relaxing" );
    gesture_prefix   = config["gesture_prefix"  ].as<std::string>( /*default=*/ "{}" );


    //-- Fill for the gesture and video prefix.
    exercises = config["exercises"].as<std::vector<std::string>>( /*default=*/ empty );


    //-- Lists of phrases for Qt to randomly pull from.
    speech_relaxation  = config["speech_relaxation" ].as<std::vector<std::string>>( /*default=*/ empty );
    speech_motivation  = config["speech_motivation" ].as<std::vector<std::string>>( /*default=*/ empty );
    speech_faster      = config["speech_faster"     ].as<std::vector<std::string>>( /*default=*/ empty );
    speech_slower      = config["speech_slower"     ].as<std::vector<std::string>>( /*default=*/ empty );
    emotion_motivation = config["emotion_motivation"].as<std::vector<std::string>>( /*default=*/ empty );

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

    //-- Set the initial video.
    this->sleepThread(1.0);
    std::string msg = fmt::format("set video {}", fmt::format(video_path, video_default));
    this->sendMessage(msg, 0.5);

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
    addLoopThread(std::bind(&QtPhysioCoach::inputLoop, this), /*period=*/ 0.01, /*start=*/ true);

    return true;
}


void QtPhysioCoach::run() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();
    std::string msg;

    //-- Loop until the manager stops running.
    while (this->getManagerRunning()) {
        
        //-- If this thread is active, run.
        if (this->getThreadStatus(thread_id)) {

            //-- Calibrate for the users resting heart-rate.
            this->calibrate();


            msg = fmt::format("set audio {}", 
                fmt::format(audio_path, 
                fmt::format(audio_exercise_base, audio_suffix[speed_idx])
            )); 
            this->sendMessage(msg, 1.0);


            // 1) marching -- 3 minutes.
            msg = fmt::format("set speech {}",
                "You can stand up now. "
                "The first exercise will help to get you warmed up! "
                "Follow my lead with this marching exercise."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 180.0);


            // 2) step-up -- 2 minutes.
            msg = fmt::format("set speech {}",
                "That was excellent. Let's keep you moving with "
                "some step-up, reach and pulls for two minutes."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[1], 120.0);
            

            // 3) marching -- 1 minutes.
            msg = fmt::format("set speech {}",
                "You're doing fantastic. "
                "Let's go back to marching for a little bit."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 60.0);


            // 4) lateral -- 2 minutes.
            msg = fmt::format("set speech {}",
                "Nice! Push hard on these laterals "
                "for two minutes."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[2], 120.0);


            // 5) marching -- 1 minutes.
            msg = fmt::format("set speech {}",
                "You're doing amazing. Back to marching for one minute."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 60.0);


            // 6) both arms -- 2 minutes.
            msg = fmt::format("set speech {}",
                "Try out this both arms forward "
                "exercise for the next two minutes."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[3], 120.0);


            // 7) marching -- 1 minutes.
            msg = fmt::format("set speech {}",
                "That was really good. Back to marching for one minute again."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 60.0);


            // 8) both lateral -- 2 minutes.
            msg = fmt::format("set speech {}",
                "Let's get moving again with some laterals for two minutes."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[2], 120.0);
            

            // 9) marching -- 1 minutes.
            msg = fmt::format("set speech {}",
                "Excelente! Let's calm down with some marching for one minute."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 60.0);


            //TODO: Cool Down.

            // 10) step-up -- 2 minutes.
            msg = fmt::format("set speech {}", fmt::format(
                "Home stretch {}. Let's start cooling down by "
                "returning to the step-up, reach and pulls for two minutes."
                , this->part_name
            ));
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[1], 120.0);


            // 11) marching -- 3 minutes.
            msg = fmt::format("set speech {}",
                "Last but not least, we'll finish off with some "
                "marching for the last three minutes."
            );
            this->sendMessage(msg, 5.0);
            this->runExercise(exercises[0], 180.0);


            msg = fmt::format("set speech {}", fmt::format(
                "Amazing work {}. Thank you for exercising with me today."
                , this->part_name
            ));
            this->sendMessage(msg, 10.0);


            this->sendMessage("exit");

            this->close();

        } else {
            this->sleepThread(0.1);
        }
    }

    return;
}


void QtPhysioCoach::calibrate() {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();
    std::string msg;
    this->sleepThread(1.0);

    //-- Start the relaxing audio and video.
    msg = fmt::format("set audio {}", fmt::format(audio_path, audio_relaxing));
    this->sendMessage(msg, 0.5);

    msg = fmt::format("set video {}", fmt::format(video_path, video_relaxing));
    this->sendMessage(msg, 0.5);

    msg = fmt::format("set gesture {} {}", fmt::format(gesture_prefix, gesture_relaxing), 0.5);
    this->sendMessage(msg, 0.5);

    if (!this->calib_skip) {

        //-- Give some instructional speech.
        msg = fmt::format("set speech {}", fmt::format(
            "Hello {}, my name is QT and I will be your "
            "personal trainer for today's session. "
            "For the next {} minutes, I am going to "
            "read your heart-rate. This will give me a good idea "
            "of the range to keep you active in. "
            "For now, just sit back, listen to this nice music, "
            "and take big breaths. Let's get started!"
            , this->part_name, (int)(this->calib_time/60)
        ));
        this->sendMessage(msg, 15.0);


        //-- Start the clocks.
        auto start = std::chrono::system_clock::now();
        auto event = std::chrono::system_clock::now();
        auto clock = std::chrono::system_clock::now();

        // Clear out the inbox after the instructions have finished.
        inbox.clear();

        while (true) {
            
            //-- Get the time and see if we're done calibrating.
            auto current = std::chrono::system_clock::now();
            std::chrono::duration<double> dur = current - start;
            if (dur.count() > calib_time || !this->getThreadStatus(thread_id)) {
                break;
            }


            //-- Tell the user how much time is left.
            dur = current - clock;
            if (dur.count() > 60.0) {

                //-- Reset time since last clock check.
                clock = std::chrono::system_clock::now();

                dur = current - start;
                size_t time_left = (int)(this->calib_time/60) - (int)(dur.count()/60);

                msg = fmt::format("set speech {}", fmt::format(
                    "You're doing great {}. "
                    "Only {} minute{} left."
                    , (time_left % 2) ? this->part_name : ", don't forget to breathe"
                    , time_left
                    , ((time_left > 1) ? "s" : "")
                ));
                this->sendMessage(msg, 5.0);
            }


            //-- Give some random speech or emotion.
            dur = current - event;
            if (dur.count() > 30.0) {

                //-- Reset time since last event.
                event = std::chrono::system_clock::now();

                //-- Choose an emotion to display 50% of the time.
                size_t what = rand() % 2;
                if (what) { //1
                    msg = fmt::format("set emotion {}", 
                        hriPhysio::chooseRandom(this->emotion_motivation)
                    );
                } else { //0
                    msg = fmt::format("set speech {}",
                        hriPhysio::chooseRandom(this->speech_relaxation)
                    );
                }
                this->sendMessage(msg, 5.0);
            }

            //-- Wait some time between loops.
            this->sleepThread(1.0);
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
    msg = fmt::format("set video {}", fmt::format(video_path, video_default));
    this->sendMessage(msg, 1.0);

    //-- Congratulate user. Tell them between 40 and 70
    msg = fmt::format("set speech {}", fmt::format(
        "My calibrations are complete! Thank you for your patience. "
        "I measured your resting heart-rate to be {:.1f} beats per minute. "
        "With this I am going to do my best to get your "
        "heart-rate between {:.1f} and {:.1f} beats per minute."
        , HRresting, HRR_40, HRR_70
    ));
    this->sendMessage(msg, 30.0);

    return;
}


void QtPhysioCoach::runExercise(const std::string typeExercise, const double seconds) {

    //-- Get the id for the current thread.
    const std::thread::id thread_id = std::this_thread::get_id();
    std::chrono::duration<double> dur;

    std::string msg;
    msg = fmt::format("set video {}", fmt::format(video_path, typeExercise));
    this->sendMessage(msg, 0.5);
    bool video_playing = true;

    msg = fmt::format("set speech {}",
        "Try to imitate the video, while keeping pace with the music"
    );
    this->sendMessage(msg, 5.0);

    msg = fmt::format("set gesture {} {}", 
        fmt::format(gesture_prefix, typeExercise), 
        speed_modifier[speed_idx]
    );
    this->sendMessage(msg, 0.5);


    //-- Start the clocks.
    auto start = std::chrono::system_clock::now();
    auto rule  = std::chrono::system_clock::now();
    auto event = std::chrono::system_clock::now();

    //-- Clear out the inbox after the instructions have finished.
    inbox.clear();

    while (true) {
        
        //-- Get the time and see if we're done calibrating.
        auto current = std::chrono::system_clock::now();
        dur = current - start;
        if (dur.count() > seconds || !this->getThreadStatus(thread_id)) {
            break;
        }

        //-- Only play the video for the first few seconds.
        if (dur.count() > video_time && video_playing) {

            msg = fmt::format("set video {}", fmt::format(video_path, video_default));
            this->sendMessage(msg, 0.5);

            video_playing = false;
        }


        //-- Check if it's time to perform a rule update.
        dur = current - rule;
        if (dur.count() > 30.0) {

            std::cerr << "[DEBUG] " //TODO: REMOVE.
                      << "Updating rule." 
                      << std::endl;

            //-- Reset the rule timer.
            rule = std::chrono::system_clock::now();

            lock.lock();
            std::vector<double> HRbuffer(inbox.size());
            inbox.dequeue(HRbuffer.data(), inbox.size());
            lock.unlock();

            
            bool changed = false;
            double HRexercise = hriPhysio::mean(HRbuffer);

            if (HRexercise < HRR_40) {
                //Increase the speed!!
                if (speed_idx < speed_modifier.size()-1) {
                    speed_idx += 1;
                    changed = true;

                    msg = fmt::format("set speech {}",
                        hriPhysio::chooseRandom(speech_faster)
                    );
                    this->sendMessage(msg, 0.5);
                }
            } else if (HRexercise > HRR_70) {
                //Decrease the speed!!
                if (speed_idx > 0) {
                    speed_idx -= 1;
                    changed = true;

                    msg = fmt::format("set speech {}",
                        hriPhysio::chooseRandom(speech_slower)
                    );
                    this->sendMessage(msg, 0.5);
                }
            } else {

                msg = fmt::format("set speech {}", 
                    hriPhysio::chooseRandom(speech_motivation)
                );
                this->sendMessage(msg, 5.0);
            }

            //-- If the speed_idx changed, update the current audio.
            if (changed) {
                msg = fmt::format("set audio {}", 
                    fmt::format(audio_path, fmt::format(audio_exercise_base, audio_suffix[speed_idx])
                ));
                this->sendMessage(msg, 5.0);
            }
        }


        //-- Give some random speech or emotion.
        dur = current - event;
        if (dur.count() > 20.0) {

            //-- Reset time since last event.
            event = std::chrono::system_clock::now();

            //-- Choose something for the robot to do.
            size_t what = rand() % 3;
            if (what == 2)  {

                lock.lock();
                std::vector<double> HRbuffer(inbox.size());
                inbox.dequeue(HRbuffer.data(), inbox.size(), inbox.size());
                lock.unlock();

                double HRexercise = hriPhysio::mean(HRbuffer);
                msg = fmt::format("set speech {}", fmt::format(
                    "Your heart-rate is {:.1f}. Keep it up!", 
                    HRexercise
                ));

            } else if (what == 1) {

                msg = fmt::format("set emotion {}", 
                    hriPhysio::chooseRandom(this->emotion_motivation)
                );

            } else { //0

                msg = fmt::format("set speech {}",
                    hriPhysio::chooseRandom(this->speech_motivation)
                );

            }
            this->sendMessage(msg, 5.0);
        }


        //-- Wait some time between loops.
        this->sleepThread(1.0);
    }

    return;
}


void QtPhysioCoach::sendMessage(const std::string& message, const double sleep_post/*=0.0*/) {
    
    std::cerr << "[SENDING] " << message << std::endl;

    std_msgs::String msg;
    msg.data = message;
    qt_controller_pub.publish(msg); 

    if (sleep_post > 0.0) {
        this->sleepThread(sleep_post);
        std::cerr << "[DEBUG] Finished sleeping..." << std::endl;
    }

    return;
}


void QtPhysioCoach::inputLoop() {
    ros::spinOnce();
    return;
}


void QtPhysioCoach::inputCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    
    lock.lock();
    inbox.enqueue(msg->data.data(), msg->data.size());
    lock.unlock();
    
    ROS_INFO("Inbox len %ld", inbox.size());

    return;
}
