import pandas as pd
import csv
import os
import sys
import numpy as np

filename = os.path.join(sys.path[0], "csv/Sun Nov 15 18_02_39 2020.csv")
print(filename)

df = pd.read_csv(filename)
# print(df.shape)
var_ax = df['ax'].var()
var_ay = df[' ay'].var()
var_az = df[' az'].var()

var_gx = df[' gx'].var()
var_gy = df[' gy'].var()
var_gz = df[' gz'].var()

rxs = df[' rx'].to_numpy()
rys = df[' ry'].to_numpy()
rzs = df[' rz'].to_numpy()

unrolled_rxs = []
unrolled_rys = []
unrolled_rzs = []

offset = 0
for i in range(1, len(rxs) - 1):
    if abs(rxs[i] - rxs[i - 1]) > 359:
        if rxs[i] < 0:
            offset = offset + 180
        else:
            offset = offset - 180
    unrolled_rxs.append(rxs[i] + offset)

offset = 0
for i in range(1, len(rys) - 1):
    if abs(rys[i] - rys[i - 1]) > 359:
        if rys[i] < 0:
            offset = offset + 180
        else:
            offset = offset - 180
    unrolled_rys.append(rys[i] + offset)

offset = 0

for i in range(1, len(rzs) - 1):
    if abs(rzs[i] - rzs[i - 1]) > 359:
        if rzs[i] < 0:
            offset = offset + 180
        else:
            offset = offset - 180
    unrolled_rzs.append(rzs[i] + offset)

var_rx = np.nanvar(unrolled_rxs)
var_ry = np.nanvar(unrolled_rys)
var_rz = np.nanvar(unrolled_rzs)

print(var_ax, var_ay, var_az)
print(var_gx, var_gy, var_gz)
print(var_rx, var_ry, var_rz)


