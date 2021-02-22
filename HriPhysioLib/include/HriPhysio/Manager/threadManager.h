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

#ifndef HRI_PHYSIO_MANAGER_THREAD_MANAGER_H
#define HRI_PHYSIO_MANAGER_THREAD_MANAGER_H

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Manager {
        class ThreadManager;
    }
}

class hriPhysio::Manager::ThreadManager {
private:
    //-- A pool for threads, their running status, and the managers status.
    std::vector< std::thread > pool;
    std::map< std::thread::id, bool > status;
    std::atomic< bool > running;

    //-- Mutex for ensuring atomicity.
    std::mutex lock;

    
public:
    ThreadManager();

    ~ThreadManager();

    std::thread::id addThread(std::function<void(void)> func, const bool start=true);

    std::thread::id addLoopThread(std::function<void(void)> func, const double period=0.0, const bool start=true);

    void interruptThread(const std::thread::id thread_id);

    bool getThreadStatus(const std::thread::id thread_id);
    
    bool getManagerRunning();

    void start();

    void stop();

    void close();

    void wait();


protected:
    //virtual bool threadInit();


private:
    void setThreadStatus(const bool thread_status);
    void looperWrapper(std::function<void(void)> func, const double period);


public:
    //-- Disallow copy and assignment operators.
    ThreadManager(const ThreadManager&) = delete;
    ThreadManager &operator=(const ThreadManager&) = delete;
};

#endif /* HRI_PHYSIO_MANAGER_THREAD_MANAGER_H */
