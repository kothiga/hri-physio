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

#ifndef HRI_PHYSIO_PROCESSING_BUTTERWORTH_LOW_PASS_H
#define HRI_PHYSIO_PROCESSING_BUTTERWORTH_LOW_PASS_H

#include <cmath>
#include <memory>

#include <HriPhysio/Processing/biquadratic.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        class ButterworthLowPass;
    }
}

class hriPhysio::Processing::ButterworthLowPass : public hriPhysio::Processing::Biquadratic {
public:

    /* ============================================================================
    **  Main Constructor.
    ** 
    ** @param rate    Sampling-rate of the provided signal.
    ** ============================================================================ */
    ButterworthLowPass(const unsigned int rate);


    /* ============================================================================
    **  Calculates the coefficients for a butterworth low pass filter.
    **
    ** @param freq    The center frequency used to calculate filter coefficients.
    ** ============================================================================ */
    void updateCoefficients(const double freq);
};

#endif /* HRI_PHYSIO_PROCESSING_BUTTERWORTH_LOW_PASS_H */
