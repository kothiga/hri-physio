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

from audioPlayer import AudioPlayer

import rospy
from std_msgs.msg import String

import argparse

def get_args():
    parser = argparse.ArgumentParser(description='audio_streamer')
    parser.add_argument('-n', '--name',  default="audio_streamer",          help='Name for the module.                  (default: {})'.format("audio_streamer"))
    parser.add_argument('-r', '--rate',  default=44100,  type=int,          help='Sampling rate.                        (default: {})'.format(44100))
    parser.add_argument('-c', '--chan',  default=2,      type=int,          help='Number of Audio Channels in Array.    (default: {})'.format(2))
    parser.add_argument('-C', '--chunk', default=1024,   type=int,          help='Chunk size to stream data.            (default: {})'.format(1024))
    parser.add_argument('-l', '--loop',  default=True, action='store_false',help='Loop the last track when finished.    (default: {})'.format(True))
    parser.add_argument('-f', '--fade',  default=1.0,    type=float,        help='Duration to fade between song change. (default: {})'.format(1.0))
    args = parser.parse_args()
    return args


# Global variable.
audio_name = ""

def callback(msg):
    global audio_name
    audio_name = msg.data
    return


def listener():

    # Parse some arguments.
    args = get_args()

    # Set up the node.
    rospy.init_node('{}'.format(args.name), anonymous=True)
    rospy.Subscriber('{}/audio_name'.format(args.name), String, callback)
    rate = rospy.Rate(100)


    # Set up the audio player.
    player = AudioPlayer(args)


    # Get a reference to the global var.
    global audio_name
    prev_name = ""

    # Loop while things are okay.
    while not rospy.is_shutdown():

        # Check if the audio_name changed.
        if audio_name != prev_name:

            # Keep track of the last audio_name.
            prev_name = audio_name

            # Fade out current track.
            player.fadeOut(args.fade)

            # Check for break out name.
            if audio_name.lower() == "exit" or audio_name.lower() == "quit":
                player.closeStream()
                rospy.signal_shutdown("Shutting down Audio Streamer...")
                exit(0)

            # Open the new track.
            if player.openAudio(audio_name, fade=args.fade):
                print("Loaded file ``{}`` successfully!!".format(audio_name))
            else:
                continue

        else:
            # Sleep for a short time.
            rate.sleep()
        
        # Loop while the audio is available.
        while player.play():

            # If the audio_name changed break out.
            if audio_name != prev_name:
                break

        if args.loop and audio_name == prev_name:
            player.resetTrack()

    return

if __name__ == '__main__':
    listener()
