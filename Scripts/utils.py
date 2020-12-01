import os
import sys
import csv
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

def get_basepath():
    return sys.path[0][:-7]

def plot2d(xys, labels, title):
    mpl.rcParams['legend.fontsize'] = 10
    for i in range(len(xys)):
        xs, ys = xys[i]
        plt.plot(xs, ys, label=labels[i])
    plt.legend()
    plt.title(title)
    plt.show()
    pass

def plot3d(xyzs, labels, title):
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for i in range(len(xyzs)):
        xs, ys, zs = xyzs[i]
        ax.plot(xs, ys, zs, label=labels[i])
    ax.legend()
    plt.title(title)
    plt.show()
    pass