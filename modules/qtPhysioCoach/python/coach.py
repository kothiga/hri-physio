#!/usr/bin/env python

# ================================================================================
# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
#     University of Waterloo, All rights reserved.
# 
# Authors: 
#     Austin Kothig <austin.kothig@uwaterloo.ca>
#     Alperen Akgun <sami.alperen.akgun@uwaterloo.ca>
# 
# CopyPolicy: Released under the terms of the BSD 3-Clause License. 
#     See the accompanying LICENSE file for details.
# ================================================================================

#import numpy as np

import time

import rospy
import sys
from std_msgs.msg import String, Float32

class Coach(object):

    def __init__(self, args):


        self.sub = rospy.Subscriber("chatter",String,self.callback_hr)
        self.pub = rospy.Publisher("/number_count", Float32, queue_size=100)

        self.hr_buffer = []


    def callback_hr(self, msg):
        print('--', type(msg.data), msg.data)
        self.hr_buffer.append(msg.data)

    def 


    def calibrate(self):

        t = time.time()
        while time.time()-t < 20:

            time.sleep(1)
            self.pub.publish('')


        return


    def runExercise(self, idx, speed):



        return


    