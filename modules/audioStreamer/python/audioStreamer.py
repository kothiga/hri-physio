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

from librosa.core import audio
import rospy
from std_msgs.msg import String

import pyaudio
import soundfile as sf
import numpy as np
import argparse

# Global variable.
audio_name = ""

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


class AudioPlayer(object):

    def __init__(self, args):

        # Set up the audio stream.
        self.sampling_rate = args.rate
        self.num_channels  = args.chan
        
        self.audio_device = None
        self.audio_stream = None
        self.openStream(self.sampling_rate, self.num_channels)

        # Current data and sampling rate.
        self.data     = np.zeros(shape=(0,self.num_channels))
        self.data_len = 0
        self.rate     = 0
        
        # Player options.
        self.window  = 0
        self.chunk   = args.chunk
        
        return


    def resetTrack(self):
        self.window = 0
        return


    def fadeOut(self, duration):
        
        # Generate the fade out.
        samples_left = self.data_len - self.window
        num_samples  = np.min((int(duration * self.rate), samples_left))
        fade_out     = np.linspace(1.0, 0.0, num=num_samples)
        
        # Fade the data out.
        self.data[self.window : self.window+num_samples] *= fade_out[:,None]

        # Trim off the remaining samples.
        self.data_len = self.window + num_samples

        # Play out the rest of the sample.
        while self.play(): None

        return


    def play(self):

        # Check if a file is loaded, or state is playing.
        if self.data_len <= 0: return False

        # Get the next samples chunk.
        chunk_size = min(self.chunk, max(0,self.data_len-self.window))

        # If at the end of the window, let user know.
        if chunk_size == 0: return False

        # Copy the data.
        samples = np.zeros((self.chunk, self.num_channels), dtype=np.float32)        
        samples[:chunk_size] = self.data[self.window : self.window+chunk_size]
        
        # Advance window.
        self.window += self.chunk
        
        # Write to the stream.
        self.audio_stream.write(samples.tobytes())

        return True


    def openAudio(self, filename):
        
        # Reset the position of the window.
        self.window = 0
        try:
            self.data, self.rate = sf.read(filename)
            self.data_len = len(self.data)

        except RuntimeError:
            print("Audio file ``{}`` could not be opened!!".format(filename))
            self.data     = np.zeros(shape=(0,self.num_channels))
            self.data_len = 0
            self.rate     = 0
            
            return False
        
        return True

    
    def openStream(self, rate, chan):

        if self.audio_device != None:
            self.closeStream()
        
        self.audio_device = pyaudio.PyAudio()
        self.audio_stream = self.audio_device.open(
            format=pyaudio.paFloat32,
            channels=chan,
            rate=rate,
            output=True
        )
        
        return


    def closeStream(self):

        # Clean up the audio_stream.
        self.audio_stream.stop_stream()
        self.audio_stream.close()
        self.audio_device.terminate()

        self.audio_stream = None
        self.audio_device = None
        
        return


def callback(data):
    global audio_name
    audio_name = data.data
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
            if player.openAudio(audio_name):
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
