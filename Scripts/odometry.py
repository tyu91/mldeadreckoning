import pandas as pd
import csv
import os
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

filename = os.path.join(sys.path[0][:-7], "data/imu_vs/velocity with Average Window Sun Nov 15 17_52_12 2020.csv")

print(filename)

df = pd.read_csv(filename)
print(df.shape)

dt = 0.5 #1Hz

vxs = []
vys = []
vzs = []

with open(filename) as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        try:
            vxs.append(float(row[0]))
            vys.append(float(row[1]))
            vzs.append(float(row[2]))
        except:
            # sometimes the last row is incomplete
            pass

pos_x = []
pos_y = []
pos_z = []

#initial pose
pos_x.append(0)
pos_y.append(0)
pos_z.append(0)

for i in range(len(vxs)):
    pos_x.append(pos_x[i] + vxs[i]*dt)
    pos_y.append(pos_y[i] + vys[i]*dt)
    pos_z.append(pos_z[i] + vzs[i]*dt)

t = np.arange(0, len(pos_x))

mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pos_x, pos_y,label='position curve')
ax.legend()
plt.show()


