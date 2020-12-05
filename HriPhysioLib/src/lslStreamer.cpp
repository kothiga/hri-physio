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

#include <HriPhysio/Stream/lslStreamer.h>

using namespace hriPhysio::Stream;


LslStreamer::LslStreamer() : 
    StreamerInterface() {

}


LslStreamer::~LslStreamer() {

}


bool LslStreamer::openInputStream() {

    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::RECEIVER;





    return true;
}


bool LslStreamer::openOutputStream() {
    
    if (this->mode != modeTag::NOTSET) {
        return false;
    }

    this->mode = modeTag::SENDER;



    

    return true;
}


void LslStreamer::publish(const std::vector<hriPhysio::varType>&  buff, std::size_t channels) {

    // int8_t,int16_t,int32_t,int64_t,long long,float,double

    switch (var) {
    case hriPhysio::varTag::INT8:
        this->pushStream<int8_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT16:
        this->pushStream<int16_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT32:
        this->pushStream<int32_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT64:
        this->pushStream<int64_t>(buff, channels);
        break;
    case hriPhysio::varTag::LONGLONG:
        this->pushStream<long long>(buff, channels);
        break;
    case hriPhysio::varTag::FLOAT:
        this->pushStream<float>(buff, channels);
        break;
    case hriPhysio::varTag::DOUBLE:
        this->pushStream<double>(buff, channels);
        break;
    default:
        break;
    }
}


void LslStreamer::receive(std::vector<hriPhysio::varType>& buff, std::size_t channels) {


    switch (var) {
    case hriPhysio::varTag::INT8:
        this->pullStream<int8_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT16:
        this->pullStream<int16_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT32:
        this->pullStream<int32_t>(buff, channels);
        break;
    case hriPhysio::varTag::INT64:
        this->pullStream<int64_t>(buff, channels);
        break;
    case hriPhysio::varTag::LONGLONG:
        this->pullStream<long long>(buff, channels);
        break;
    case hriPhysio::varTag::FLOAT:
        this->pullStream<float>(buff, channels);
        break;
    case hriPhysio::varTag::DOUBLE:
        this->pullStream<double>(buff, channels);
        break;
    default:
        break;
    }
}


template<typename T>
void LslStreamer::pushStream(const std::vector<hriPhysio::varType>&  buff, std::size_t channels) {

    std::vector<T> vecOut;
    
    for (std::size_t idx = 0; idx < buff.size(); ++idx) {
        vecOut.push_back( std::get<T>(buff[idx]) );
    }

    // push stream.
}


template<typename T>
void LslStreamer::pullStream(std::vector<hriPhysio::varType>& buff, std::size_t channels) {

    std::vector<T> vecIn;

    // receive from stream.

    for (std::size_t idx = 0; idx < vecIn.size(); ++idx) {
        buff[idx] = vecIn[idx];
    }
}
