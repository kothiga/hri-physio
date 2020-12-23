import neurokit2 as nk
import pandas as pd
import matplotlib.pyplot as plt


#df = pd.read_csv('ppg-sit.csv')
#df = pd.read_csv('ppg-walkinroom.csv')
#df = pd.read_csv('ppg-sit-upper-arm.csv')
df = pd.read_csv('ppg-walkinroom-upper-arm.csv')



ppg = df['ppg2'].values
ppg_clean = nk.ppg_clean(ppg, sampling_rate=1000)
info = nk.ppg_findpeaks(ppg_clean)
peaks = info["PPG_Peaks"]
plt.plot(ppg, label="raw PPG") 
plt.plot(ppg_clean, label="clean PPG") 
plt.scatter(peaks, ppg[peaks], c="r", label="systolic peaks") 
plt.legend() 
plt.show()


#signals, info = nk.ppg_process(ppg, sampling_rate=130)
#nk.ppg_plot(signals)
#plt.show()

#ppg = nk.ppg_simulate(heart_rate=75,  duration=30)
#print(ppg)
