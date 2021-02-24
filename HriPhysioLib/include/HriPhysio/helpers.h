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

#ifndef HRI_PHYSIO_HELPERS_H
#define HRI_PHYSIO_HELPERS_H

#include <iostream>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

namespace hriPhysio {

    /* ============================================================================
    **  Helpful classes for parsing and others single tasks.
    ** ============================================================================ */
    class ArgParser;
    

    /* ============================================================================
    **  Initialize a common variant type to support relevant data types.
    ** ============================================================================ */
    using varType = std::variant<char,int16_t,int32_t,int64_t,float,double>;
    enum  varTag { CHAR, INT16, INT32, INT64, LONGLONG, FLOAT, DOUBLE };
    
    struct printVisitor {
        void operator()(char    v ) const { std::cout << "char("    << v << ")"; }
        void operator()(int16_t v ) const { std::cout << "int16_t(" << v << ")"; }
        void operator()(int32_t v ) const { std::cout << "int32_t(" << v << ")"; }
        void operator()(int64_t v ) const { std::cout << "int64_t(" << v << ")"; }
        void operator()(float   v ) const { std::cout << "float("   << v << ")"; }
        void operator()(double  v ) const { std::cout << "double("  << v << ")"; }
    };


    /* ============================================================================
    **  Methods for Math.
    ** ============================================================================ */
    const double pi    = 2 * acos(0.0); //-- High precision pi.
    const double sqrt2 = sqrt(2.0);     //-- High precision sqrt(2.0)


    /* ============================================================================
    **  Methods for Strings.
    ** ============================================================================ */
    std::vector< std::string > parseString(std::string& str);
    std::vector< double > toVecDouble(const std::vector< std::string >& source, size_t idx=0);
    std::string combineString(const std::vector< std::string >& source, size_t idx=0);
    void toLower(std::string& str);
    void toUpper(std::string& str);
}


class hriPhysio::ArgParser {
    /* ============================================================================
    **  Adapted from users iain and 0x90
    **    https://stackoverflow.com/questions/865668/
    ** ============================================================================ */
public:
    ArgParser(int &argc, char **argv) {
        for (int i = 1; i < argc; ++i) {
            this->tokens.push_back(std::string(argv[i]));
        }
    }
    const std::string& getCmdOption(const std::string &option) const {
        std::vector<std::string>::const_iterator itr;
        itr = std::find(this->tokens.begin(), this->tokens.end(), option);
        if (itr != this->tokens.end() && ++itr != this->tokens.end()){
            return *itr;
        }
        static const std::string empty_string("");
        return empty_string;
    }
    const bool getCmdOption_asBool(const std::string &option) const {
        std::string str(getCmdOption(option));
        hriPhysio::toUpper(str);
        return (str == "TRUE" || str == "YES" || str == "1" || str == "ENABLE") ? true : false;
    }
    const double getCmdOption_asDouble(const std::string &option) const {
        return std::stod( getCmdOption(option) );
    }
    const float getCmdOption_asFloat(const std::string &option) const {
        return std::stof( getCmdOption(option) );
    }
    const int getCmdOption_asInt(const std::string &option) const {
        return std::stoi( getCmdOption(option) );
    }
    const long getCmdOption_asLong(const std::string &option) const {
        return std::stol( getCmdOption(option) );
    }
    const long double getCmdOption_asLongDouble(const std::string &option) const {
        return std::stold( getCmdOption(option) );
    }
    const long long getCmdOption_asLongLong(const std::string &option) const {
        return std::stoll( getCmdOption(option) );
    }
    const unsigned long getCmdOption_asUnsignedLong(const std::string &option) const {
        return std::stoul( getCmdOption(option) );
    }
    const unsigned long long getCmdOption_asUnsignedLongLong(const std::string &option) const {
        return std::stoull( getCmdOption(option) );
    }
    bool cmdOptionExists(const std::string &option) const {
        return std::find(
            this->tokens.begin(), this->tokens.end(), option
        ) != this->tokens.end();
    }
private:
    std::vector<std::string> tokens;
};

#endif /* HRI_PHYSIO_HELPERS_H */
