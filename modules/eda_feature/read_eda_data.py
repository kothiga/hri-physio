import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.dates import MinuteLocator, DateFormatter
import datetime as dt


def read_eda_data(url, column_names):

    df_1 = pd.read_csv(url, header = 1, skiprows = [2], sep='\t', usecols=column_names)
    df = df_1.rename({column_names[0]: "timestamp", column_names[1]: "eda_data" }, axis='columns')
    df['timestamp'] = pd.to_datetime(df['timestamp'],unit='ms')

    position = df.columns.get_loc('timestamp')
    df['time_elapsed'] = df.iloc[1:, position] - df.iat[0, position]
    seconds = df.time_elapsed.dt.total_seconds() 

    return df, seconds


