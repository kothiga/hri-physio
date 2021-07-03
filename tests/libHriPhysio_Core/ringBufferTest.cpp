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

#include <HriPhysio/Core/ringBuffer.h>

TEST_CASE("Test Single enqueue Function and size Function") {

    hriPhysio::Core::RingBuffer<int> rb(3);
    //rb.setWarnings(true);

    CHECK(rb.size() == 0);

    rb.enqueue(5);
    CHECK(rb.size() == 1);

    rb.enqueue(3);
    CHECK(rb.size() == 2);

    rb.enqueue(9);
    CHECK(rb.size() == 3);

    rb.enqueue(13); //overwrite 5.
    CHECK(rb.size() == 3);
}

TEST_CASE("Test Single enqueue Function and Single dequeue Function") {

    hriPhysio::Core::RingBuffer<int> rb(3);
    rb.setWarnings(true);
    int out;

    rb.enqueue(5);
    rb.dequeue(out);
    CHECK(out == 5);
    CHECK(rb.size() == 0);

    rb.enqueue(3);
    rb.enqueue(9);
    rb.enqueue(13);
    rb.enqueue(22); //overwrites 3.
    CHECK(rb.size() == 3);

    rb.dequeue(out);
    CHECK(out == 9);
    CHECK(rb.size() == 2);

    rb.dequeue(out);
    CHECK(out == 13);
    CHECK(rb.size() == 1);

    rb.dequeue(out);
    CHECK(out == 22);
    CHECK(rb.size() == 0);
}