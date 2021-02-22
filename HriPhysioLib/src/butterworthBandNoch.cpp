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

#include <HriPhysio/Processing/butterworthBandNoch.h>

using namespace hriPhysio::Processing;


ButterworthBandNoch::ButterworthBandNoch(const unsigned int rate, const double width) : 
    Biquadratic(rate, width) {
    
}


void ButterworthBandNoch::updateCoefficients(const double freq) {

    //-- Allocate some local variables.
    double c, d;

    //-- Compute the coeff for the given center frequency.
    c  =  tan( hriPhysio::pi * (band_width / sampling_rate) );
    d  =  2.0 * cos( 2.0 * hriPhysio::pi * (freq / sampling_rate) );
    a0 =  1.0 / (1.0 + c);
    a1 = -a0 * d;
    a2 =  a0;
    b1 = -a0 * d;
    b2 =  a0 * (1.0 - c);

    //-- Cache this center frequency, 
    //-- so that it is not recomputed.
    center_frequency = freq;
    
    return;
}
