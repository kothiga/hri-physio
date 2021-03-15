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

import os
import random
import sys
import time

import numpy as np
import pandas as pd
from pylsl import StreamInfo, StreamOutlet, local_clock

# These have to come after pylsl
import neurokit2 as nk
import matplotlib.pyplot as plt

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

    def pos_int(value):
        ivalue = int(value)
        if ivalue < 0:
            raise argparse.ArgumentTypeError("%s is an invalid positive int value" % value)
        return ivalue

    parser = argparse.ArgumentParser(
        prog='neurokitSimulator',
        description='',
        usage='python neurokitSimulator.py'
    )
    parser.add_argument('--conf',       default=None,                                                            help='Override argparse with a yaml configuration. Location of a config file. (default: {})'.format(None))
    parser.add_argument('--name',       default='neuroSim',                                                      help='Name for the module prefix.                                             (default: {})'.format('neuroSim'))
    parser.add_argument('--type',       default='ecg',           choices=['ecg','ppg','rsp','eda','emg', 'lin'], help='The type of signal to synthesize.                                       (default: {})'.format('ecg'))
    parser.add_argument('--chunk',      default=32,              type=pos_int,                                   help='Size of data chunks to transmit.                                        (default: {})'.format(32))
    parser.add_argument('--seconds',    default=10,              type=pos_int,                                   help='How many seconds of synthetic signal to simulate.                       (default: {})'.format(10))
    parser.add_argument('--rate',       default=128,             type=pos_int,                                   help='Sampling rate of the synthetic signal.                                  (default: {})'.format(128))
    parser.add_argument('--noise',      default=0.01,            type=float,                                     help='The amount of noise that should be added to signal.                     (default: {})'.format(0.01))
    parser.add_argument('--bpm',        default=70,              type=pos_int,                                   help='Beats per minute for signal. Used in ECG and PPG.                       (default: {})'.format(70))
    parser.add_argument('--ecg-method', default='ecgsyn',        choices=['ecgsyn','simple'],                    help='The model used to generate the ECG signal.                              (default: {})'.format('ecgsyn'))
    parser.add_argument('--resp',       default=15,              type=pos_int,                                   help='Respiratory rate (breath cycle per second). Used in RSP.                (default: {})'.format(15))
    parser.add_argument('--rsp-method', default='breathmetrics', choices=['breathmetrics', 'sinusoidal'],        help='The model used to generate the RSP signal.                              (default: {})'.format('breathmetrics'))
    parser.add_argument('--scr',        default=3,               type=pos_int,                                   help='Number of skin conductance responses. Used in EDA.                      (default: {})'.format(3))
    parser.add_argument('--drift',      default=-0.01,           type=float,                                     help='Slope of a linear drift of the signal. Used in EDA.                     (default: {})'.format(-0.01))
    parser.add_argument('--bnum',       default=3,               type=pos_int,                                   help='Number of bursts of activity to simulate. Used in EMG.                  (default: {})'.format(3))
    parser.add_argument('--bdur',       default=1.0,             type=float,                                     help='Duration of the simulated bursts. Used in EMG.                          (default: {})'.format(1.0))
    parser.add_argument('--loop',       default=False,           action='store_true',                            help='Enable loop until keyboard interrupt.                                   (default: {})'.format(False))
    parser.add_argument('--plot',       default=False,           action='store_true',                            help='Plot the generated signal as it\'s transmitted.                         (default: {})'.format(False))
    parser.add_argument('--verbose',    default=False,           action='store_true',                            help='Print out log information.                                              (default: {})'.format(False))
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
        try: args.type       = conf['type']
        except KeyError: pass
        try: args.chunk      = conf['chunk']
        except KeyError: pass
        try: args.seconds    = conf['seconds']
        except KeyError: pass
        try: args.rate       = conf['rate']
        except KeyError: pass
        try: args.noise      = conf['noise']
        except KeyError: pass
        try: args.bpm        = conf['bpm']
        except KeyError: pass
        try: args.ecg_method = conf['ecg-method']
        except KeyError: pass
        try: args.resp       = conf['resp']
        except KeyError: pass
        try: args.rsp_method = conf['rsp-method']
        except KeyError: pass
        try: args.scr        = conf['scr']
        except KeyError: pass
        try: args.drift      = conf['drift']
        except KeyError: pass
        try: args.bnum       = conf['bnum']
        except KeyError: pass
        try: args.bdur       = conf['bdur']
        except KeyError: pass
        try: args.loop       = conf['loop']
        except KeyError: pass
        try: args.plot       = conf['plot']
        except KeyError: pass
        try: args.verbose    = conf['verbose']
        except KeyError: pass

    # Do some error checking for the strings.
    errOccurr = False
    if args.type.upper() not in ['ECG', 'PPG', 'RSP', 'EDA', 'EMG', 'LIN']:
        if not errOccurr: # Check to see if this is the first error.
            parser.print_help()
            errOccurr = True
        print(" ")
        errPrint("\n")
        print("  Type ``{}`` is not supported. Please provide the program with one of the following types: ecg, ppg, rsp, eda, emg".format(args.type))
        print(" ")      
    if args.ecg_method.upper() not in ['ECGSYN', 'SIMPLE']:
        if not errOccurr:
            parser.print_help()
            errOccurr = True
        print(" ")
        errPrint("\n")
        print("  ECG-Method ``{}`` is not supported. Please provide the program with one of the following types: ecgsyn, simple".format(args.ecg_meth))
        print(" ")
    if args.rsp_method.upper() not in ['BREATHMETRICS', 'SINUSOIDAL']:
        if not errOccurr:
            parser.print_help()
            errOccurr = True
        print(" ")
        errPrint("\n")
        print("  RSP-Method ``{}`` is not supported. Please provide the program with one of the following types: breathmetrics, sinusoidal".format(args.rsp_meth))
        print(" ")
    if errOccurr:
        print("  Examples:")
        print("\t python neurokitSimulator.py --type ecg --bpm 90 --ecg-method ecgsyn")
        print("\t python neurokitSimulator.py --type ppg --bpm 90")
        print("\t python neurokitSimulator.py --type rsp --resp 40 --rsp-method breathmetrics")
        print("\t python neurokitSimulator.py --type eda --scr 5 --drift -0.04")
        print("\t python neurokitSimulator.py --type emg --bnum 4 --bdur 0.7")
        print("\t python neurokitSimulator.py --type lin --seconds 10 --rate 128")
        print(" ")
        exit()

    return args


# ===================
# Main Implementation
# ===================

class neurokitSimulator(object):
    """
    A simple streamer which encapsulates the use of neurokit2's
    simulate methods for various physiological signals.  
    Synthetic data is then forwarded out onto lab streaming layer.
    """
    def __init__(self, args):

        # Store the args used after init.
        self.name    = args.name
        self.type    = args.type.upper()
        self.chunk   = args.chunk
        self.rate    = args.rate
        self.loop    = args.loop
        self.plot    = args.plot
        self.verb    = args.verbose
        

        # Set the time for sleeping between chunks.
        self.sleep_for = self.chunk / self.rate

        # Initialize the information for the stream.
        self.info = StreamInfo(
            name           = "/{}/{}".format(self.name, self.type),
            channel_count  = 1, 
            nominal_srate  = self.rate, 
            channel_format = 'double64', 
            source_id      = 'neurokitStreamer0'
        )

        # Open an outlet for the stream.
        self.outlet = StreamOutlet(self.info, chunk_size=self.chunk, max_buffered=360)


        # Create a simulated signal using the appropriate neurokit method.
        self.simulated = None
        if self.type == 'ECG':
            self.simulated = nk.ecg_simulate(
                duration      = args.seconds, 
                sampling_rate = self.rate,
                noise         = args.noise,
                heart_rate    = args.bpm,
                method        = args.ecg_method
            )
        if self.type == 'PPG':
            self.simulated = nk.ppg_simulate(
                duration       = args.seconds,
                sampling_rate  = self.rate,
                ibi_randomness = args.noise,
                heart_rate     = args.bpm
            )
        if self.type == 'RSP':
            self.simulated = nk.rsp_simulate(
                duration         = args.seconds,
                sampling_rate    = self.rate,
                noise            = args.noise,
                respiratory_rate = args.resp,
                method           = args.rsp_method
            )
        if self.type == 'EDA':
            self.simulated = nk.eda_simulate(
                duration      = args.seconds,
                sampling_rate = self.rate,
                noise         = args.noise,
                scr_number    = args.scr,
                drift         = args.drift
            )
        if self.type == 'EMG':
            self.simulated = nk.emg_simulate(
                duration       = args.seconds,
                sampling_rate  = self.rate,
                noise          = args.noise,
                burst_number   = args.bnum,
                burst_duration = args.bdur
            )
        if self.type == 'LIN':
            self.simulated = np.arange(
                start = 1.0, 
                stop  = float(args.seconds * self.rate)+1.0,
                step  = 1.0
            )

        # Init a matplotlib figure if enabled.
        self.fig = None
        if self.plot:
            self.fig = plt.figure()

        return


    def run(self):

        self.wait_for_connection()

        while True:
            try:
                for idx in range(0, len(self.simulated), self.chunk):

                    # Get a chunk and write it out.
                    current_chunk = self.simulated[idx:idx+self.chunk]
                    self.outlet.push_chunk(current_chunk)
                    
                    if self.verb:
                        print( current_chunk.dtype )
                        print(local_clock(), current_chunk[:], sep='\n')

                    if self.plot:
                        data_len  = self.chunk*10
                        data_plot = self.simulated[idx:idx+data_len]
                        # Pad the remainder to maintain length.
                        if len(data_plot) < data_len:
                            data_plot = np.append(data_plot, np.zeros(data_len - len(data_plot)))
                        
                        self.fig.clear()
                        ax0 = self.fig.add_subplot(111)
                        ax0.plot(data_plot)
                        plt.pause(0.0001)

                    # Sleep at the end of a chunk for simulating real time.
                    time.sleep(self.sleep_for)

                # Break out if we're not looping.
                if not self.loop:
                    break
            
            except KeyboardInterrupt:
                # Catch the interrupt and clean everything up.
                errPrint("Caught keyboard interrupt.")
                exit()

        return


    def wait_for_connection(self):
        try:
            # Some ASCII animation.
            waiting = [' \\ ', ' ─ ', ' / ', ' | ']
            count = 0

            # Loop until a consumer is connected the the outlet.
            while not self.outlet.wait_for_consumers(timeout=1.5):
                print("\r", "  Wating for consumers . . . {}".format(waiting[count%4]), end="")
                count += 1
            print("\r", "  Consumer connected!"," "*10)

        except KeyboardInterrupt:
            # Catch the interrupt and clean everything up.
            errPrint("Caught keyboard interrupt.")
            exit()
            
        return


    def cleanup(self):
        print("\nCleaning up . . . ")
        return


# ===================
# Program Entry Point
# ===================

def main():

    # Get parameters from a user.
    args = get_args()

    # Flush args to the screen.
    print("\n")
    print(" "*11, "[Neurokit Simulator]"); print(" "*2, "="*38)
    print(" -- Stream Name        : {}".format("/{}/{}".format(args.name, args.type.upper())))
    print(" -- Chunk Size         : {}".format(args.chunk))
    print(" -- Seconds of Data    : {}".format(args.seconds))
    print(" -- Sampling Rate      : {}".format(args.rate))
    print(" -- Signal Noise       : {}".format(args.noise))
    if args.type.upper() == "ECG":
        print(" -- Beats per Minute   : {}".format(args.bpm))
        print(" -- ECG Synth Method   : {}".format(args.ecg_method))
    if args.type.upper() == "PPG":
        print(" -- Beats per Minute   : {}".format(args.bpm))
    if args.type.upper() == "RSP":
        print(" -- Respiratory Rate   : {}".format(args.resp))
        print(" -- RSP Synth Method   : {}".format(args.rsp_method))
    if args.type.upper() == "EDA":
        print(" -- Skin Conductance   : {}".format(args.scr))
        print(" -- Slope of Lin Drift : {}".format(args.drift))
    if args.type.upper() == "EMG":
        print(" -- Number of Bursts   : {}".format(args.bnum))
        print(" -- Duration of Bursts : {}".format(args.bdur))
    print(" -- Loop when finished : {}".format("ENABLED" if args.loop else "DISABLED"))
    print(" -- Plot Data Sent     : {}".format("ENABLED" if args.plot else "DISABLED"))
    print(" -- Verbose Output     : {}".format("ENABLED" if args.verbose else "DISABLED"))
    print("\n")
    
    
    # Initialize the stream simulator module.
    simStream = neurokitSimulator(args)

    # Begin running the module, and clean up when finished.
    try:
        simStream.run()

    finally:
        simStream.cleanup()

    return


if __name__ == "__main__":
    main()
