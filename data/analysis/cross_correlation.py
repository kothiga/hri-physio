# -*- coding: utf-8 -*-
"""
@author: jessys
#email: jj5song@uwaterloo.ca 

"""
import numpy as np
import pandas as pd

def normalized_correlation(a, b): 
    a = (a - np.mean(a)) / (np.std(a) * len(a))
    b = (b - np.mean(b)) / (np.std(b))
    return np.correlate(a, b)

def normalized_correlation_full(a, b): 
    a = (a - np.mean(a)) / (np.std(a) * len(a))
    b = (b - np.mean(b)) / (np.std(b))
    return np.correlate(a, b, 'full')

# replace '' with file path
recorded_file = pd.read_csv('')
realtime_file = pd.read_csv('')

# replace column name with the metric (ex. sdnn, rmssd, hf...) 
recorded = recorded_file['sdnn'].tolist()
realtime = realtime_file['sdnn'].tolist()
# calculate correlation with same length
recorded = recorded[:len(realtime)]

corr_full = np.correlate(realtime, recorded, 'full')
normalized_correlation_full = normalized_correlation_full(realtime, recorded)
corr = np.correlate(realtime, recorded)
normalized_corr = normalized_correlation(recorded, realtime)
print(corr)
print(normalized_corr)

corr_csv = pd.DataFrame({'corr_full': corr_full, 
                       'corr_full_normalized': normalized_correlation_full })
corr_csv.to_csv(r'correlation.csv', index=False)