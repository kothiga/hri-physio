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

#include <doctest.h>

#include <iostream>
#include <vector>


void printVector(const std::vector<double>& vec) {
    std::cout << "[";
    for (std::size_t idx = 0; idx < vec.size(); ++idx) {
        if (idx) std::cout << ",";
        std::cout << vec[idx];
    } std::cout << "] " << std::endl;
}


void printVector(const std::vector<std::vector<double>>& vec) {
    std::cout << "[";
    for (std::size_t idx = 0; idx < vec.size(); ++idx) {
        if (idx) std::cout << "," << std::endl;
        std::cout << "[";
        for (std::size_t jdx = 0; jdx < vec[idx].size(); ++jdx) {
            if (jdx) std::cout << ",";
            std::cout << vec[idx][jdx];
        }
        std::cout << "]";
    } 
    std::cout << "] " << std::endl;
}


double absError(const std::vector<double>& vec1, const std::vector<double>& vec2) {
    double total_error = 0;
    for (std::size_t idx = 0; idx < vec1.size(); ++idx) {
        total_error += abs(vec1[idx] - vec2[idx]);
    }
    return total_error;
}
