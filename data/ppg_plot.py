import neurokit2 as nk
import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy

numpy.set_printoptions(threshold=sys.maxsize)

# Simulate data
ppg = nk.ppg_simulate(duration=10, sampling_rate=1000, heart_rate=70)
print(max(ppg))

# Process signal
signals, info = nk.ppg_process(ppg, sampling_rate=1000)

# Plot
nk.ppg_plot(signals)
plt.show()
