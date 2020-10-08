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

void tempA() {
    std::cout << "tempA" << std::endl;
}

PhysioManager::PhysioManager() {

    addLoopThread(tempA, 1.5, false);
    addLoopThread(std::bind(&PhysioManager::tempB, this), 1.2, false);
}


PhysioManager::~PhysioManager() {

}

void PhysioManager::interactive() {
    std::string str;
    while (true) {
        std::cin >> str;
        std::cout << ">> " << str << std::endl;

        if (str == "exit") {
            break;
        }
    }
}


bool PhysioManager::threadInit() {
    return true;
}


//void PhysioManager::tempA() {
//    std::cout << "tempA" << std::endl;
//}


void PhysioManager::tempB() {
    std::cout << "tempB" << std::endl;
}
