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

#include <HriPhysio/Processing/biquadratic.h>

using namespace hriPhysio::Processing;


Biquadratic::Biquadratic(const unsigned int rate, const double width/*=0.0*/) : 
    sampling_rate(rate),
    band_width(width),
    center_frequency(0.0) {
    
}


void Biquadratic::filter(const double* source, double* target, const std::size_t numSamples, const double freq) {

    if (center_frequency != freq) {
        updateCoefficients(freq);
        center_frequency = freq;  //-- Cache the cf used to prevent recalculating coeff.  
    }

    //-- Filter the signal.
    bilinearTransformation(source, target, numSamples);

    return;
}


void Biquadratic::setSamplingRate(const unsigned int rate) {
    
    sampling_rate = rate;
    center_frequency = 0.0; //-- Reset cf to force recalculation of coeff.
    
    return;
}


void Biquadratic::setBandWidth(const double width) {
    
    band_width = width;
    center_frequency = 0.0; //-- Reset cf to force recalculation of coeff.

    return;
}


void Biquadratic::bilinearTransformation(const double* source, double* target, const std::size_t numSamples) {
    
    //-- Filter buffer.
	double p0i = 0.0, p1i = 0.0, p2i = 0.0;
	double p0o = 0.0, p1o = 0.0, p2o = 0.0;

	/* ============================================================================
	**  Begin running the single bilinear transform on the provided input data.
	**  The resulting filtered data is stored in the provided double array.
	** ============================================================================ */

	for (std::size_t sample = 0; sample < numSamples; sample++) {

		p0i = source[sample];
		p0o = (a0*p0i + a1*p1i + a2*p2i) - (b2*p2o + b1*p1o);

		p2i = p1i; p1i = p0i;
		p2o = p1o; p1o = p0o;

		target[sample] = p0o;
	}

    return;
}
