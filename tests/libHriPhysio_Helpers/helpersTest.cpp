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

#include <HriPhysio/helpers.h>

TEST_CASE("Test toLower Function") {

    //-- Test a string that is all lowercase.
    std::string s1("the colour of a restless night more than a faint memory can provide");
    hriPhysio::toLower(s1);
    CHECK(s1 == "the colour of a restless night more than a faint memory can provide");

    //-- Test a string that is all uppercase.
    std::string s2("IF YEARS GO BY FEAR AND PAIN WILL SUSTAIN IN YOUR EYES");
    hriPhysio::toLower(s2);
    CHECK(s2 == "if years go by fear and pain will sustain in your eyes");

    //-- Test a mixed string with punctuation and special characters.
    std::string s3("SH3 m@Y Nev3r r3tuRN, but I can wAiT f0revEr!!");
    hriPhysio::toLower(s3);
    CHECK(s3 == "sh3 m@y nev3r r3turn, but i can wait f0rever!!");
}

TEST_CASE("Test toUpper Function") {

    //-- Test a string that is all lowercase.
    std::string s1("as for the boy it pleased me to know");
    hriPhysio::toUpper(s1);
    CHECK(s1 == "AS FOR THE BOY IT PLEASED ME TO KNOW");

    //-- Test a string that is all uppercase.
    std::string s2("THAT HE WILL JUST WAKE UP NINE YEARS OLD ALONE AND AFRAID");
    hriPhysio::toUpper(s2);
    CHECK(s2 == "THAT HE WILL JUST WAKE UP NINE YEARS OLD ALONE AND AFRAID");

    //-- Test a mixed string with punctuation and special characters.
    std::string s3("He'Ll l1ve foR tHe d@y th1s m0mEnt com3s ar0und ag4in!?");
    hriPhysio::toUpper(s3);
    CHECK(s3 == "HE'LL L1VE FOR THE D@Y TH1S M0MENT COM3S AR0UND AG4IN!?");

}

TEST_CASE("Variant") {

    //-- Test the size of variant
    std::cout << "Size of varType (<int8_t,int16_t,int32_t,int64_t,float,double>) ";
    hriPhysio::varType v1;
    std::cout << sizeof(v1) << std::endl;

    int8_t t8 = 5;
    std::cout << "sizeof(int8_t) " << sizeof(t8) << std::endl;

    int16_t t16 = 5;
    std::cout << "sizeof(int16_t) " << sizeof(t16) << std::endl;

    int32_t t32 = 5;
    std::cout << "sizeof(int32_t) " << sizeof(t32) << std::endl;

    int64_t t64 = 5;
    std::cout << "sizeof(int64_t) " << sizeof(t64) << std::endl;

    float f = 5.f;
    std::cout << "sizeof(float) " << sizeof(f) << std::endl;

    double d = 5.0123;
    std::cout << "sizeof(double) " << sizeof(d) << std::endl;


    v1 = (double) 5.0021;
    std::cout << "double " << std::get<double>(v1) << std::endl;

    v1 = (int64_t) 5123465789;
    std::cout << "int64_t "; std::visit(hriPhysio::printVisitor(), v1); 
    std::cout << std::endl;
}
