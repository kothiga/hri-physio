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

#include <HriPhysio/helpers.h>


void stringTransform(std::string& str, int (*func)(int)) {
    std::string::iterator it = str.begin();
    while (it != str.end()) {
        *it = (*func)(*it);
        ++it;
    }
}


void hriPhysio::toLower(std::string& str) { 
    stringTransform(str, std::tolower);
}


void hriPhysio::toUpper(std::string& str) { 
    stringTransform(str, std::toupper);
}
