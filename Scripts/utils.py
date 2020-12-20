import os
import sys
import csv
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import argparse


CSVIDXMAP = {
            "ax": 0,
            "ay": 1,
            "az": 2,
            "gx": 3,
            "gy": 4,
            "gz": 5,
            "rx": 6,
            "ry": 7,
            "rz": 8,
            "lat": 9,
            "long": 10,
            "alt": 11,
            "time": 12,
            "velocity": 13,
            "gps_velx": 14,
            "gps_vely": 15,
            "gps_velz": 16, 
            "gps_posx": 17, 
            "gps_posy": 18, 
            "gps_posz": 19
        }

def get_basepath():
    return sys.path[0][:-7]

def plot2d(xys, labels, title, colors=None, show_plots=True, savefig=False):
    mpl.rcParams['legend.fontsize'] = 10
    for i in range(len(xys)):
        xs, ys = xys[i]
        if colors is None:
            plt.plot(xs, ys, label=labels[i])
        else:
            plt.plot(xs, ys, c=colors[i], label=labels[i])
    plt.legend()
    plt.title(title)
    if savefig:
        filename = title + ".png"
        full_filename = os.path.join(get_basepath(), "data", "figures", filename)
        plt.savefig(full_filename)
    if show_plots:
        plt.show()
    plt.close()

def plot3d(xyzs, labels, title, show_plots=True, savefig=False):
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for i in range(len(xyzs)):
        xs, ys, zs = xyzs[i]
        ax.plot(xs, ys, zs, label=labels[i])
    ax.legend()
    plt.title(title)
    if savefig:
        filename = title + ".png"
        full_filename = os.path.join(get_basepath(), "data", "figures", filename)
        plt.savefig(full_filename)
    if show_plots:
        plt.show()
    plt.close()
