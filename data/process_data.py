# copped from https://stackoverflow.com/questions/47210512/using-pykalman-on-raw-acceleration-data-to-calculate-position

from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import csv
import os
import sys

window_size = 50

def rotate_vector(base, rotation):
    # https://stackoverflow.com/questions/34050929/3d-point-rotation-algorithm/34060479
    
    ry = float(rotation[0]) / (180.0 / np.pi)
    rx = float(rotation[1]) / (180.0 / np.pi)
    rz = float(rotation[2]) / (180.0 / np.pi)

    bx = float(base[0])
    by = float(base[1])
    bz = float(base[2])

    cosa = np.cos(rz)
    sina = np.sin(rz)

    cosb = np.cos(ry)
    sinb = np.sin(ry)

    cosc = np.cos(rx)
    sinc = np.sin(rx)

    Axx = cosa * cosb
    Axy = cosa * sinb * sinc - sina * cosc
    Axz = cosa * sinb * cosc + sina * sinc
    Ayx = sina * cosb
    Ayy = sina * sinb * sinc + cosa * cosc
    Ayz = sina * sinb * cosc - cosa * sinc
    Azx = -sinb
    Azy = cosb * sinc
    Azz = cosb * cosc

    px = bx
    py = by
    pz = bz

    bx = Axx * px + Axy * py + Axz * pz
    by = Ayx * px + Ayy * py + Ayz * pz
    bz = Azx * px + Azy * py + Azz * pz

    return [bx, by, bz]

def plot3(xs, ys, zs, plotted_value):
    t1 = np.arange(0, len(xs))
    t2 = np.arange(0, len(ys))
    t3 = np.arange(0, len(zs))
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)

    ax1.plot(t1, xs)
    ax1.set_title("transformed " + plotted_value + "xs over time")
    ax2.plot(t2, ys)
    ax2.set_title("transformed " + plotted_value + "ys over time")
    ax3.plot(t3, zs)
    ax3.set_title("transformed " + plotted_value + "zs over time")

    plt.show()

def compute_windowed_acc_and_vel(axs, ays, azs, rxs, rys, rzs, rolling=False):
    prev_i = 0
    correction_x = 0.054
    correction_y = 0.55
    correction_z = 0.855
    axs = [float(x) + correction_x for x in axs]
    ays = [float(x) + correction_y for x in ays]
    azs = [float(x) + correction_z for x in azs]

    rxs = [float(x) for x in rxs]
    rys = [float(x) for x in rys]
    rzs = [float(x) for x in rzs]

    t_accs = []

    for i in range(len(axs)):
        try:
            t_accs.append(rotate_vector([axs[i], ays[i], azs[i]], [rxs[i], rys[i], rzs[i]]))
        except:
            pass

    t_accs = np.array(t_accs)
    axs = t_accs[:, 0]
    ays = t_accs[:, 1]
    azs = t_accs[:, 2]

    windowed_ax = []
    windowed_ay = []
    windowed_az = []
    vxs = []
    vys = []
    vzs = []

    if (rolling):
        for i in range(0, len(axs) - 1 - window_size):
            windowed_ax.append(np.average(np.array([float(x) for x in axs[i:i + window_size]])))
            windowed_ay.append(np.average(np.array([float(x) for x in ays[i:i + window_size]])))
            windowed_az.append(np.average(np.array([float(x) for x in azs[i:i + window_size]])))
            
            vxs.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in axs[i:i + window_size]])))
            vys.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in ays[i:i + window_size]])))
            vzs.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in azs[i:i + window_size]])))
    else:
        for i in range(window_size, len(axs) - 1, window_size):
            windowed_ax.append(np.average(np.array([float(x) for x in axs[prev_i:i]])))
            windowed_ay.append(np.average(np.array([float(x) for x in ays[prev_i:i]])))
            windowed_az.append(np.average(np.array([float(x) for x in azs[prev_i:i]])))
            
            vxs.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in axs[prev_i:i]])))
            vys.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in ays[prev_i:i]])))
            vzs.append(np.sum(np.array([((1.0 / window_size) * float(x) * 9.8) for x in azs[prev_i:i]])))

            prev_i = i

    axs = windowed_ax
    ays = windowed_ay
    azs = windowed_az

    return axs, ays, azs, vxs, vys, vzs

def plot_a_v(axs, ays, azs, vxs, vys, vzs):
    t1a = np.arange(0, len(axs))
    t2a = np.arange(0, len(ays))
    t3a = np.arange(0, len(azs))

    t1v = np.arange(0, len(vxs))
    t2v = np.arange(0, len(vys))
    t3v = np.arange(0, len(vzs))

    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=3, figsize=(12,8))

    ax1[0].plot(t1a, axs)
    ax1[0].set_title("transformed axs over time")
    ax1[1].plot(t2a, ays)
    ax1[1].set_title("transformed ays over time")
    ax1[2].plot(t3a, azs)
    ax1[2].set_title("transformed azs over time")

    ax2[0].plot(t1v, vxs)
    ax2[0].set_title("transformed vxs over time")
    ax2[1].plot(t2v, vys)
    ax2[1].set_title("transformed vys over time")
    ax2[2].plot(t3v, vzs)
    ax2[2].set_title("transformed vzs over time")

    plt.show()

def plot_3d(axs, ays, azs):
    axs = [float(x) for x in axs]
    ays = [float(x) for x in ays]
    azs = [float(x) for x in azs]
    mpl.rcParams['legend.fontsize'] = 10
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(axs, ays, azs, label='parametric curve')
    ax.legend()
    plt.show()

    # axs = [float(x) for x in axs]
    # ays = [float(x) for x in ays]
    # azs = [float(x) for x in azs]
    # plt.plot(ays, azs)
    # plt.show()


if __name__ == "__main__":
    # filename = os.path.join(sys.path[0], "csv", os.listdir("csv")[2])
    # filename = os.path.join(sys.path[0], "csv", "Sun Nov 15 18_01_53 2020.csv")
    # filename = os.path.join(sys.path[0], "csv", "Sun Nov 15 18_22_35 2020.csv")
    relative_filename = "Sun Nov 15 17_52_12 2020.csv"
    # relative_filename = "stationary-Thu Nov 19 14_05_11 2020.csv"
    filename = os.path.join(sys.path[0], "csv", "50hz", relative_filename)

    axs = []
    ays = []
    azs = []
    gxs = []
    gys = []
    gzs = []
    rxs = []
    rys = []
    rzs = []
    with open(filename) as csvfile:
        reader = csv.reader(csvfile)

        start = 1
        start_iter = 0
            
        for row in reader:
            if start_iter < start:
                start_iter += 1
                continue
            try:
                axs.append(row[0])
                ays.append(row[1])
                azs.append(row[2])
                gxs.append(row[3])
                gys.append(row[4])
                gzs.append(row[5])
                rxs.append(row[6])
                rys.append(row[7])
                rzs.append(row[8])
            except:
                pass
        is_rolling = False
        axs, ays, azs, vxs, vys, vzs = compute_windowed_acc_and_vel(axs, ays, azs, rxs, rys, rzs, is_rolling)
        plot_a_v(axs, ays, azs, vxs, vys, vzs)
        plot_3d(axs, ays, azs)
        # import pdb; pdb.set_trace()
        rolling_word = "with Rolling Window " if is_rolling else "with Average Window "
        with open(os.path.join(sys.path[0], "imu_vs", "velocity " + rolling_word + relative_filename), "w") as v_outfile:
            writer = csv.writer(v_outfile)
            writer.writerow(["vxs", "vys", "vzs"])
            row_vs = [list(elem) for elem in list(zip(vxs, vys, vzs))]
            writer.writerows(row_vs)

