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

#ifndef HRI_PHYSIO_PROCESSING_HILBERT_TRANSFORM_H
#define HRI_PHYSIO_PROCESSING_HILBERT_TRANSFORM_H

#include <cmath>
#include <memory>

#include <HriPhysio/Processing/PocketFFT/pocketfft.h>
#include <HriPhysio/Processing/math.h>

#include <HriPhysio/helpers.h>

namespace hriPhysio {
    namespace Processing {
        class HilbertTransform;
    }
}

class hriPhysio::Processing::HilbertTransform {
private:

    /* ============================================================================
	**  Variables received from the constructor.
	** ============================================================================ */
	std::size_t num_samples;

    
    /* ============================================================================
    **  PocketFFT variables.
    ** ============================================================================ */ 
    pocketfft::shape_t  shape;
    pocketfft::stride_t stride;
    pocketfft::shape_t  axes;

    std::vector< std::complex<double> > input;
    std::vector< std::complex<double> > data;
    std::vector< std::complex<double> > output;

    

public:

    /* ============================================================================
    **  Main Constructor.
    ** ============================================================================ */
    HilbertTransform(std::size_t samples);


    /* ===========================================================================
	**  Destructor.
	** =========================================================================== */
    ~HilbertTransform();


    /* ===========================================================================
	**  Process.
	** =========================================================================== */
    void process(const std::vector<double>& source, std::vector<double>& target);


    /* ===========================================================================
	**  Resize.
	** =========================================================================== */
    void resize(const std::size_t samples);


private:
    /* ===========================================================================
	**  Real to Complex.
	** =========================================================================== */
    void realToComplex(const double* source, std::complex<double>* target, const std::size_t num_samples);
    

    /* ===========================================================================
	**  Complex to Real.
	** =========================================================================== */
    void complexToReal(const std::complex<double>* source, double* target, const std::size_t num_samples);


    /* ===========================================================================
	**  Absolute Square.
	** =========================================================================== */
    double absoluteSquare(const std::complex<double> value);
};

#endif /* HRI_PHYSIO_PROCESSING_HILBERT_TRANSFORM_H */
