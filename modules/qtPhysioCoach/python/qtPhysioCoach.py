#!/usr/bin/env python

# ================================================================================
# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, 
#     University of Waterloo, All rights reserved.
# 
# Authors: 
#     Austin Kothig <austin.kothig@uwaterloo.ca>
# 
# CopyPolicy: Released under the terms of the BSD 3-Clause License. 
#     See the accompanying LICENSE file for details.
# ================================================================================


from pylsl import StreamInlet, resolve_stream

import rospy
from std_msgs.msg import String

import os, sys, time
import argparse
import yaml


# ===============
# Formatting Help
# ===============

def logPrint(ptype, pfile=sys.stdout, *args, **kwargs):
    print("[{}] [{}]:".format(ptype, time.asctime()), *args, file=pfile, **kwargs)
def errPrint(*args, **kwargs):
    logPrint("ERROR", sys.stderr, *args, **kwargs)
def debugPrint(*args, **kwargs):
    logPrint("DEBUG", sys.stderr, *args, **kwargs)
def infoPrint(*args, **kwargs):
    logPrint("INFO", sys.stdout,  *args, **kwargs)


# ==================================
# Argument and Configuration Parsing
# ==================================

def get_args():
    parser = argparse.ArgumentParser(description='qtPhysioCoach')
    parser.add_argument('--conf',       default=None,                      help='Override argparse with a yaml configuration.         (default: {})'.format(None))
    parser.add_argument('-n','--name',  default="qtPhysioCoach",           help='Name for the module.                                 (default: {})'.format("qtPhysioCoach"))
    parser.add_argument('-i','--input', default="PolarH10/6080292F/HR",    help='Name of the LSL input stream.                        (default: {})'.format('PolarH10/6080292F/HR'))
    parser.add_argument('-c','--calib', default=False,action='store_true', help='Calibrate the resting HR or use the default value.   (default: {})'.format(False))
    parser.add_argument('--HR',         default=70,   type=int,            help='Default resting heart-rate.                          (default: {})'.format(70))
    parser.add_argument('-a','--age',   default=25,   type=int,            help='Default age.                                         (default: {})'.format(25))
    parser.add_argument('--verbose',    default=False,action='store_true', help='Print out log information.                           (default: {})'.format(False))
    
    #parser.add_argument('-r', '--rate',  default=44100,  type=int,          help='Sampling rate.                        (default: {})'.format(44100))
    #parser.add_argument('-c', '--chan',  default=2,      type=int,          help='Number of Audio Channels in Array.    (default: {})'.format(2))
    #parser.add_argument('-C', '--chunk', default=1024,   type=int,          help='Chunk size to stream data.            (default: {})'.format(1024))
    #parser.add_argument('-l', '--loop',  default=True, action='store_false',help='Loop the last track when finished.    (default: {})'.format(True))
    #parser.add_argument('-f', '--fade',  default=1.0,    type=float,        help='Duration to fade between song change. (default: {})'.format(1.0))
    args = parser.parse_args()

    if args.conf is not None:

        # Check that the file exists.
        if args.verbose:
            infoPrint("Loading config file ``{}``.".format(args.conf))
        if not os.path.isfile(args.conf):
            errPrint("File ``{}`` not found. Returning set args.".format(args.conf))
            return args
        
        # Open and load the file.
        with open(args.conf) as f:
            conf = yaml.load(f, Loader=yaml.SafeLoader)
        
        # Overload default args from yaml if set.
        try: args.name       = conf['name']
        except KeyError: pass
        try: args.input      = conf['input']
        except KeyError: pass
        try: args.calib      = conf['calib']
        except KeyError: pass
        try: args.HR         = conf['HR']
        except KeyError: pass
        try: args.age        = conf['age']
        except KeyError: pass
                
        

    return args



def main():

    args = get_args()

    pub = rospy.Publisher('/QtController/input', String, queue_size=1000)
    rospy.init_node('{}'.format(args.name), anonymous=True)

    time.sleep(3)

    print("pubin")
    pub.publish("set speech hello, Are you ready to do some EXERCISE")

    count = 0
    while not rospy.is_shutdown():
        count += 1
        pub.publish("set speech hello world %d" % count)
        time.sleep(3)


#    print("pubed it", flush=True)
#
#    streams = resolve_stream('name', 'PolarH10/6080292F/ECG')
#
#    print("streams", flush=True)
#
#    inlet = StreamInlet(streams[0])
#
#    print("made those streams", flush=True)
#
#    while True:
#        samp, ts = inlet.pull_sample()
#        print(ts, samp)






    return

if __name__ == '__main__':
    main()
