import pandas as pd
import csv
import os
import sys

filename = os.path.join(sys.path[0], "still.csv")
print(filename)

df = pd.read_csv(filename)

print(df.shape)
var_ax = df['ax'].var()
var_ay = df[' ay'].var()
var_az = df[' az'].var()

var_gx = df[' gx'].var()
var_gy = df[' gy'].var()
var_gz = df[' gz'].var()

var_rx = df[' rx'].var()
var_ry = df[' ry'].var()
var_rz = df[' rz'].var()

print(var_ax, var_ay, var_az)
print(var_gx, var_gy, var_gz)
print(var_rx, var_ry, var_rz)


