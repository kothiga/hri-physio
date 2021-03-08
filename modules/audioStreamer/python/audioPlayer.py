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

import pyaudio
import soundfile as sf
import numpy as np

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
        
        # Fade the audio.
        self.fadeAudio(duration)

        # Trim off the remaining samples.
        samples_left = self.data_len - self.window
        num_samples  = np.min((int(duration * self.rate), samples_left))
        self.data_len = self.window + num_samples

        # Play out the rest of the sample.
        while self.play(): None

        return

    def fadeAudio(self, duration):

        # Generate the fade.
        samples_left = self.data_len - self.window
        num_samples = np.min((int(duration * self.rate), samples_left))

        fade = None
        if self.window == 0: #fade in.
            fade = np.linspace(0.0, 1.0, num=num_samples)
        else: #fade out.
            fade = np.linspace(1.0, 0.0, num=num_samples)

        # Fade the segment of data.
        self.data[self.window : self.window+num_samples] *= fade[:,None]

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


    def openAudio(self, filename, fade=0.0):
        
        # Reset the position of the window.
        self.window = 0
        try:
            self.data, self.rate = sf.read(filename)
            self.data_len = len(self.data)
            
            if fade > 0.0:
                self.fadeAudio(fade)

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
