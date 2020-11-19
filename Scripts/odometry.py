import pandas as pd
import csv
import os
import sys

filename = os.path.join(sys.path[0], "csv/imu_vs/velocity Sun Nov 15 17_52_12 2020.csv")
print(filename)

df = pd.read_csv(filename)
print(df.shape)

dt = 1 #1Hz




