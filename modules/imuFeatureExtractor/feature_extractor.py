import csv
import numpy as np
from scipy.fft import fft
from scipy.stats import kurtosis, skew
from scipy.signal import find_peaks
import matplotlib.pyplot as plt

def get_imu_data(path, separation = '\t', show_details = True, fs = 512, plotting = True):
  """
  Assumes data is separate using tabs denoted by "\t" and returns a dictionary
  of data.

  @param path: path to csv file separated by tabs
  @param separation: what the csv is separated by
  @param show_details: whether a brief data descriptor is given

  Returns dictionary corresponding to data
  """

  with open(path, newline='') as f:
    reader = csv.reader(f)
    data = [row[0].split(separation) for row in reader]
  
  data = np.array(data[1:])[:,:len(data[1])-1]

  shimmer_unit = data[0][0][0:13]

  #Creating a dictionary to better describe the data
  #Changing the names to something more understandable

  imu_data = {}
  field_units = {}
  cal_status = {}

  dp = 3 #number of decimal points

  for i in range(len(data[0])):

    if(data[0][i].endswith("UNCAL")):
      data[0][i] = data[0][i][13:len(data[0][i])-6]
      cal_status.update({data[0][i]:"Uncalibrated"})
    elif(data[0][i].endswith("CAL")):
      data[0][i] = data[0][i][13:len(data[0][i])-4]
      cal_status.update({data[0][i]:"Calibrated"})
    else:
      data[0][i] = data[0][i][13:len(data[0][i])]
      cal_status.update({data[0][i]:"Calibration Status Unknown"})

    if(i > 0):
      imu_data.update({data[0][i] : np.round(data[2:,i].astype(np.float), dp)})
    else:
      imu_data.update({data[0][i] : data[2:,i]})
      
    field_units.update({data[0][i] : data[1][i]})

  if(show_details):
    print("The following fields were found:\n")
    for key in imu_data:
      print("{} : - {}, units :{}".format (
        key, cal_status[key], field_units[key]))
      
  if(plotting == True):
    tempset = set(imu_data)
    keylist = list(tempset)
    num_dp = len(imu_data[keylist[0]]) #number of datapoints
    t = np.linspace(0.0, num_dp/fs, num_dp, endpoint=False)
    if("Accel_LN_X" in tempset):
      plt.figure(figsize=(8,4))
      plt.plot(t, imu_data["Accel_LN_X"], label = "acc_ln_x")
      plt.plot(t, imu_data["Accel_LN_Y"], label = "acc_ln_y")
      plt.plot(t, imu_data["Accel_LN_Z"], label = "acc_ln_z")
      plt.legend()
    if("Accel_WR_X" in tempset):
      plt.figure(figsize=(8,4))
      plt.plot(t, imu_data["Accel_WR_X"], label = "acc_wr_x")
      plt.plot(t, imu_data["Accel_WR_Y"], label = "acc_wr_y")
      plt.plot(t, imu_data["Accel_WR_Z"], label = "acc_wr_z")
      plt.legend()
    if("Gyro_X" in tempset):
      plt.figure(figsize=(8,4))
      plt.plot(t, imu_data["Gyro_X"], label = "gyro_x")
      plt.plot(t, imu_data["Gyro_Y"], label = "gyro_y")
      plt.plot(t, imu_data["Gyro_Z"], label = "gyro_z")
      plt.legend()
    if("Mag_X" in tempset):
      plt.figure(figsize=(8,4))
      plt.plot(t, imu_data["Mag_X"], label = "mag_x")
      plt.plot(t, imu_data["Mag_Y"], label = "mag_y")
      plt.plot(t, imu_data["Mag_Z"], label = "mag_z")
      plt.legend()

  return imu_data

def fft_energy(data):
  """
  Returns the energy using fourier coefficients given an array of data that
  is in time series format. Used solely as utility/helper function

  @param data: Time series signal

  Returns float value of energy
  """

  fft_data = fft(data)
  return np.sum([np.square(element) for element in fft_data[0:len(data//2)]])

def get_peaks(data):
  """
  Returns the number of peaks of a signal

  @param data: Time series signal

  Returns num_peaks
  """

  peaks, _ = find_peaks(data, height=0)

  return len(peaks)

def get_valleys(data):
  """
  Returns the number of valleys of a signal

  @param data: Time series signal

  Returns num_valleys
  """

  valleys, _ = find_peaks(-1*(data), height=0)

  return len(valleys)

def get_single_window_stats(data_dict):
  """
  For a single window/instance or sample of data, it returns the following
  statistical features, of the Accelerometers:

  1. Energy, 2. Mean, 3. Standard Deviation, 4. Skewness, 5. Kurtosis, 
  6. Eccentricity, 7. Correlation Coefficients, 8. IQR

  @param data_dict: Must be a dict containing relevant IMU data
              -> Must be in format consistent with shimmer extraction
  """

  dict_features = {}

  for key in data_dict:
    if key.startswith('Accel'):
      dict_features.update(
        {"Mean_{}".format(key[len(key)-4 : len(key)]) : np.mean(data_dict[key])}
        )
      dict_features.update(
        {"RMS_{}".format(key[len(key)-4 : len(key)]) : np.sqrt(np.mean(data_dict[key]**2))}
        )
      dict_features.update(
        {"STD_{}".format(key[len(key)-4 : len(key)]) : np.std(data_dict[key])}
        )
      dict_features.update(
        {"Energy_{}".format(key[len(key)-4 : len(key)]) : fft_energy(data_dict[key])}
        )
      dict_features.update(
        {"Kurtosis_{}".format(key[len(key)-4 : len(key)]) : kurtosis(data_dict[key])}
        )
      dict_features.update(
        {"Skewness_{}".format(key[len(key)-4 : len(key)]) : skew(data_dict[key])}
        )
      dict_features.update(
        {"Peaks_{}".format(key[len(key)-4 : len(key)]) : get_peaks(data_dict[key])}
        )
      dict_features.update(
        {"Valleys_{}".format(key[len(key)-4 : len(key)]) : get_valleys(data_dict[key])}
        )
      
  return dict_features

def window(data, onset, period, fs):
  """
  Returns appropriate window of data based on specified time.
  It is only meant to be a helper function

  @param data: data from which we window
  @param onset: starting index - optional
  @param period: window of period in seconds
  @param fs: sampling frequency in Hz
  """

  endpoint = int(period*fs)
  return data[onset:onset+endpoint]
  
def get_stat_features(imu_data, fs, period = 6.7, plotting=True, epochs = False):
  """
  Returns sliding window features

  @param epochs : if the data is epoched or not, not means it is windowed
  """

  dict_features = {}

  keys = []
  for key in imu_data:
    keys.append(key)

  print(keys)

  if(epochs == False):
    for i in range(len(imu_data[keys[0]])- int(1.1*fs*period)):
      temp_dict = {}
      for key in imu_data:
        temp_dict.update({key : window(imu_data[key], i, period, fs)})
      single_instance = get_single_window_stats(temp_dict)
      if(i == 0):
        for key in single_instance:
          dict_features.update({key : [single_instance[key]]})
      else:
        for key in single_instance:
          dict_features[key].append(single_instance[key])
  else:
    for i in range(len(imu_data[keys[0]])):
      temp_dict = {}
      for key, val in imu_data.items():
        temp_dict.update({key : val[i]})
      single_instance = get_single_window_stats(temp_dict) #getting stats of a single epoch
      if(i == 0):
        for key in single_instance:
          dict_features.update({key : [single_instance[key]]})
      else:
        for key in single_instance:
          dict_features[key].append(single_instance[key])

  print("\nThe features we get are :")
  for key in dict_features:
    print("{}".format(key))
  print("where LN stands for Low Noise Accelerometer and WR for Wide Range Accelerometer")

  features = []
  for key in dict_features:
    features.append(key)

  if(plotting):
    fig, axs = plt.subplots(len(features)//5, 5, figsize=(20, 3.5*len(features)//5))

    index = 0
    for i in range(len(features)//5):
      for j in range(5):
        if(epochs == False):
          axs[i, j].plot(dict_features[features[index]])
          axs[i, j].set_title(features[index])
        else:
          axs[i, j].hist(dict_features[features[index]])
          axs[i, j].set_title(features[index])
        index += 1

  return dict_features
  

def get_epochs(imu_data, onset_array, period, fs):
  """
  Retrieves epochs/data samples as per the onset array/markers specify

  @param imu_data: A dictionary of imu data as specified
  @param onset_array: An array of the onset time (s)
  @param period: length of epoch/window (s)
  @param fs: sampling frequency
  """

  dict_epochs = {}

  array_indices = [int(elem*fs) for elem in onset_array]

  for key in imu_data:
    dict_epochs.update({key : []})

  for key in dict_epochs:   
     for elem in onset_array:
       dict_epochs[key].append(window(imu_data[key], elem, period, fs))

  return dict_epochs