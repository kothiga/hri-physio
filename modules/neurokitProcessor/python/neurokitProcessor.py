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
from neurokit2 import ecg

import numpy as np
import pandas as pd
from pylsl import StreamInfo, StreamInlet, StreamOutlet, resolve_stream, local_clock

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
        prog='neurokitProcessor',
        description='',
        usage='python neurokitProcessor.py'
    )
    parser.add_argument('--conf',    default=None,                       help='Override argparse with a yaml configuration. Location of a config file. (default: {})'.format(None))
    parser.add_argument('--name',    default='neurokitProcessor',        help='Name for the module prefix.                                             (default: {})'.format('neurokitProcessor'))
    parser.add_argument('--input',   default='/stream',                  help='Name for the LSL stream to receive from.                                (default: {})'.format('/stream'))
    parser.add_argument('--buffer',  default=130,   type=pos_int,        help='Number of samples to buffer before processing.                          (default: {})'.format(130))
    parser.add_argument('--window',  default=0,     type=pos_int,        help='Number of samples to keep after processing.                             (default: {})'.format(0))
    parser.add_argument('--plot',    default=False, action='store_true', help='Display the window of processed data.                                   (default: {})'.format(False))
    parser.add_argument('--verbose', default=False, action='store_true', help='Print out log information.                                              (default: {})'.format(False))
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
        try: args.buffer     = conf['buffer']
        except KeyError: pass
        try: args.buffer     = conf['window']
        except KeyError: pass
        try: args.plot       = conf['plot']
        except KeyError: pass
        try: args.verbose    = conf['verbose']
        except KeyError: pass

  
    return args


# ===================
# Main Implementation
# ===================

class neurokitProcessor(object):
    """
    A simple data processing module which encapsulates the use of neurokit2's
    processing methods for computing features of ecg data.
    Processed elements are forwarded out onto lab streaming layer.
    """

    # Lookup for pylsl dtypes and conversions for numpy.
    conv = {
        1:'float32', 2:'float64', 3:'string', 4:'int32',   
        5:'int16',   6:'int8',    7:'int64',  0:''
    }

    def __init__(self, args):

        # Store the args used after init.
        self.name   = args.name
        self.input  = args.input
        self.size   = args.buffer
        self.window = args.window
        self.plot   = args.plot
        self.verb   = args.verbose
        

        # Open up the input stream.
        if self.verb: infoPrint("Looking for stream ``{}``".format(self.input))
        streams    = resolve_stream('name', '{}'.format(self.input))
        self.inlet = StreamInlet(streams[0])
        if self.verb: infoPrint("Connected!!")


        # Get some info about the connected stream.
        info = self.inlet.info()
        self.rate = info.nominal_srate()
        self.fmt  = info.channel_format()

        if self.verb: print(info.as_xml())

        
        # Initialize the RMSSD output streams.
        info_rmssd = StreamInfo(
            name           = "/{}/{}".format(self.name, 'rmssd'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor0'
        )
        self.rmssd_outlet = StreamOutlet(info_rmssd)

        # Initialize the SDNN output streams.
        info_sdnn = StreamInfo(
            name           = "/{}/{}".format(self.name, 'sdnn'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor1'
        )
        self.sdnn_outlet = StreamOutlet(info_sdnn)

        # Initialize the pNN50 output streams.
        info_pnn50 = StreamInfo(
            name           = "/{}/{}".format(self.name, 'pnn50'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor2'
        )
        self.pnn50_outlet = StreamOutlet(info_pnn50)

        # Initialize the LF output streams.
        info_lf = StreamInfo(
            name           = "/{}/{}".format(self.name, 'lf'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor3'
        )
        self.lf_outlet = StreamOutlet(info_lf)

        # Initialize the HF output streams.
        info_hf = StreamInfo(
            name           = "/{}/{}".format(self.name, 'hf'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor4'
        )
        self.hf_outlet = StreamOutlet(info_hf)

        # Initialize the LF/HF output streams.
        info_lfhf = StreamInfo(
            name           = "/{}/{}".format(self.name, 'lf/hf'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor5'
        )
        self.lfhf_outlet = StreamOutlet(info_lfhf)

        # Initialize the Heart Rate output streams.
        info_hr = StreamInfo(
            name           = "/{}/{}".format(self.name, 'hr'),
            channel_count  = 1, 
            nominal_srate  = 0, # irregular sampling rate 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor6'
        )
        self.hr_outlet = StreamOutlet(info_hr)

        # Initialize the ECG-Clean output streams.
        info_ecg_clean = StreamInfo(
            name           = "/{}/{}".format(self.name, 'ecg-clean'),
            channel_count  = 1, 
            nominal_srate  = self.rate, 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor7'
        )
        self.ecg_clean_outlet = StreamOutlet(info_ecg_clean, chunk_size=self.size, max_buffered=self.size*2)

        # Initialize the Peaks output streams.
        info_peaks = StreamInfo(
            name           = "/{}/{}".format(self.name, 'peaks'),
            channel_count  = 1, 
            nominal_srate  = self.rate, 
            channel_format = 'double64', 
            source_id      = 'neurokitProcessor8'
        )
        self.peaks_outlet = StreamOutlet(info_peaks, chunk_size=self.size, max_buffered=self.size*2)


        # Buffer to store received data.
        self.buffer = np.array([], dtype=self.conv[self.fmt])
        self.stamps = np.array([], dtype='float')
      
        
        # Init a matplotlib figure if enabled.
        self.fig = None
        if self.plot:
            self.fig = plt.figure()

        return


    def run(self):

        while True:
            try:
                # Get a chunk of data.
                chunk, ts = self.inlet.pull_chunk()

                if ts:
                    # If it's good add it to the buffers.
                    self.buffer = np.concatenate([self.buffer, chunk], axis=None)
                    self.stamps = np.concatenate([self.stamps, ts],    axis=None)
                    if self.verb: infoPrint('Got something ({}/{})'.format(len(self.buffer),self.size))

                # Try to do some work.
                self.process()

            except KeyboardInterrupt:
                # Catch the interrupt and clean everything up.
                errPrint("Caught keyboard interrupt.")
                exit()

        return

    
    def process(self):

        # Check to see if we can do some work.
        if len(self.buffer) >= self.size:

            # Pull out the data.
            data = self.buffer[:self.size]
            ts   = self.stamps[:self.size]

            # Discard elements from the buffer.
            self.buffer = self.buffer[self.size-self.window:]
            self.stamps = self.stamps[self.size-self.window:]


            # Find the peaks from the received ecg data.
            peaks, info = nk.ecg_peaks(data, sampling_rate=int(self.rate))
            hrv         = nk.hrv(peaks, sampling_rate=int(self.rate))

            # Where do the peaks occur?
            r_peaks = info['ECG_R_Peaks']

            # Get the HR and clean ECG
            signals, info = nk.ecg_process(data, sampling_rate=int(self.rate))

            
            # All hrv keys:
            #   'HRV_RMSSD', 'HRV_MeanNN', 'HRV_SDNN', 'HRV_SDSD', 'HRV_CVNN',
            #   'HRV_CVSD', 'HRV_MedianNN', 'HRV_MadNN', 'HRV_MCVNN', 'HRV_IQRNN',
            #   'HRV_pNN50', 'HRV_pNN20', 'HRV_TINN', 'HRV_HTI', 'HRV_ULF', 'HRV_VLF',
            #   'HRV_LF', 'HRV_HF', 'HRV_VHF', 'HRV_LFHF', 'HRV_LFn', 'HRV_HFn',
            #   'HRV_LnHF', 'HRV_SD1', 'HRV_SD2', 'HRV_SD1SD2', 'HRV_S', 'HRV_CSI',
            #   'HRV_CVI', 'HRV_CSI_Modified', 'HRV_PIP', 'HRV_IALS', 'HRV_PSS',
            #   'HRV_PAS', 'HRV_GI', 'HRV_SI', 'HRV_AI', 'HRV_PI', 'HRV_C1d', 'HRV_C1a',
            #   'HRV_SD1d', 'HRV_SD1a', 'HRV_C2d', 'HRV_C2a', 'HRV_SD2d', 'HRV_SD2a',
            #   'HRV_Cd', 'HRV_Ca', 'HRV_SDNNd', 'HRV_SDNNa', 'HRV_ApEn', 'HRV_SampEn'
            rmssd = np.nan_to_num(hrv['HRV_RMSSD'].to_numpy())
            sdnn  = np.nan_to_num(hrv['HRV_SDNN'].to_numpy())
            pNN50 = np.nan_to_num(hrv['HRV_pNN50'] .to_numpy())
            lf    = np.nan_to_num(hrv['HRV_LF'].to_numpy())
            hf    = np.nan_to_num(hrv['HRV_HF'].to_numpy())
            lfhf  = np.nan_to_num(hrv['HRV_LFHF'].to_numpy())
            

            # All signals keys:
            #   'ECG_Raw', 'ECG_Clean', 'ECG_Rate', 'ECG_Quality', 'ECG_R_Peaks',
            #   'ECG_P_Peaks', 'ECG_Q_Peaks', 'ECG_S_Peaks', 'ECG_T_Peaks',
            #   'ECG_P_Onsets', 'ECG_T_Offsets', 'ECG_Phase_Atrial',
            #   'ECG_Phase_Completion_Atrial', 'ECG_Phase_Ventricular',
            #   'ECG_Phase_Completion_Ventricular'
            heart_rate = [signals['ECG_Rate'].to_numpy().mean()]
            ecg_clean  = signals['ECG_Clean'].to_numpy()
            peaks      = peaks.to_numpy().reshape(-1)


            if self.verb: 
                print("{}\t {}".format('HRV_RMSSD',  rmssd))
                print("{}\t {}".format('HRV_SDNN',   sdnn))
                print("{}\t {}".format('HRV_pNN50',  pNN50))
                print("{}\t {}".format('HRV_LF',     lf))
                print("{}\t {}".format('HRV_HF',     hf))
                print("{}\t {}".format('HRV_LFHF',   lfhf))

                print("{}\t {}".format('ECG_Rate',    heart_rate))
                print("{}\t {}".format('ECG_Clean',   ecg_clean))
                print("{}\t {}".format('ECG_R_Peaks', peaks))


            # Publish data.
            self.rmssd_outlet.push_sample(rmssd,   timestamp=ts[0])
            self.sdnn_outlet.push_sample(sdnn,     timestamp=ts[0])
            self.pnn50_outlet.push_sample(pNN50,   timestamp=ts[0])
            self.lf_outlet.push_sample(lf,         timestamp=ts[0])
            self.hf_outlet.push_sample(hf,         timestamp=ts[0])
            self.lfhf_outlet.push_sample(lfhf,     timestamp=ts[0])
            self.hr_outlet.push_sample(heart_rate, timestamp=ts[0])

            self.ecg_clean_outlet.push_chunk(ecg_clean, timestamp=ts[0])
            self.peaks_outlet.push_chunk(peaks,         timestamp=ts[0])
            
            # TODO: Fix timestamps
            #print("Timestamps:")
            #for t in ts[:5]:
            #    print("{:10.8f}".format(t))


            if self.plot:
                self.fig.clear()
                ax0 = self.fig.add_subplot(111)
                ax0.plot(ts, data)

                if len(r_peaks):
                    p = [ data[v] for v in r_peaks ]
                    t = [ ts[v]   for v in r_peaks ]
                    if self.verb: infoPrint(list(zip(t,p)))
                    ax0.scatter(t, p)

                plt.pause(0.0001)

        return


    def cleanup(self):
        print("\nCleaning up . . . ")
        self.inlet.close_stream()
        return



# ===================
# Program Entry Point
# ===================

def main():

    # Get parameters from a user.
    args = get_args()

#    # Flush args to the screen.
#    print("\n")
#    print(" "*11, "[Neurokit Simulator]"); print(" "*2, "="*38)
#    print(" -- Stream Name        : {}".format("/{}/{}".format(args.name, args.type.upper())))
#    print(" -- Chunk Size         : {}".format(args.chunk))
#    print(" -- Seconds of Data    : {}".format(args.seconds))
#    print(" -- Sampling Rate      : {}".format(args.rate))
#    print(" -- Signal Noise       : {}".format(args.noise))
#    if args.type.upper() == "ECG":
#        print(" -- Beats per Minute   : {}".format(args.bpm))
#        print(" -- ECG Synth Method   : {}".format(args.ecg_method))
#    if args.type.upper() == "PPG":
#        print(" -- Beats per Minute   : {}".format(args.bpm))
#    if args.type.upper() == "RSP":
#        print(" -- Respiratory Rate   : {}".format(args.resp))
#        print(" -- RSP Synth Method   : {}".format(args.rsp_method))
#    if args.type.upper() == "EDA":
#        print(" -- Skin Conductance   : {}".format(args.scr))
#        print(" -- Slope of Lin Drift : {}".format(args.drift))
#    if args.type.upper() == "EMG":
#        print(" -- Number of Bursts   : {}".format(args.bnum))
#        print(" -- Duration of Bursts : {}".format(args.bdur))
#    print(" -- Loop when finished : {}".format("ENABLED" if args.loop else "DISABLED"))
#    print(" -- Plot Data Sent     : {}".format("ENABLED" if args.plot else "DISABLED"))
#    print(" -- Verbose Output     : {}".format("ENABLED" if args.verbose else "DISABLED"))
#    print("\n")
    
    
    # Initialize the stream simulator module.
    nkProcessor = neurokitProcessor(args)

    # Begin running the module, and clean up when finished.
    try:
        nkProcessor.run()

    finally:
        nkProcessor.cleanup()

    return


if __name__ == "__main__":
    main()
