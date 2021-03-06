# copped from https://stackoverflow.com/questions/47210512/using-pykalman-on-raw-acceleration-data-to-calculate-position

from pykalman import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import csv
import os
import sys

from utils import *

def rotate_vector(base, rotation):
    """takes in base x, y, z acceleration and applies rotation. 

    Taken from https://stackoverflow.com/questions/34050929/3d-point-rotation-algorithm/34060479
    
    :param base: list of x, y, z accelerations as floats
    :param rotation: list of x, y, z rotations as floats
    :rtype: list of rotated base vector
    """
    
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

def parse_input_file(filename):
    """ parses accs and rots from csv file

    :param filename: input filename
    :rtype: 6-tuple of accs (x, y, z) and rots (x, y, z)
    """
    axs = []
    ays = []
    azs = []
    rxs = []
    rys = []
    rzs = []
    with open(filename) as csvfile:
        reader = csv.reader(csvfile)

        start = 1
        start_iter = 0
        
        # parse in data from csv file
        for row in reader:
            # allows you to truncate the first start number of samples
            if start_iter < start:
                start_iter += 1
                continue
            try:
                axs.append(float(row[0]))
                ays.append(float(row[1]))
                azs.append(float(row[2]))
                rxs.append(float(row[6]))
                rys.append(float(row[7]))
                rzs.append(float(row[8]))
            except:
                pass

    return axs, ays, azs, rxs, rys, rzs

def write_output_file(output_filename, vxs, vys, vzs):
    """ writes velocities to output csv

    :param output_filename: file to output to
    :rtype: 6-tuple of accs (x, y, z) and rots (x, y, z)
    """
    with open(output_filename, "w") as v_outfile:
        writer = csv.writer(v_outfile)
        writer.writerow(["vxs", "vys", "vzs"])
        row_vs = [list(elem) for elem in list(zip(vxs, vys, vzs))]
        writer.writerows(row_vs)

def compute_transformed_accelerations(axs, ays, azs, rxs, rys, rzs):
    """ transforms accelerations by rotations

    :rtype: tuple of transformed accelerations with shape len(axs) for each axis
    """
    t_accs = []

    # compute transformed accelerations
    for i in range(len(axs)):
        try:
            t_accs.append(rotate_vector([axs[i], ays[i], azs[i]], [rxs[i], rys[i], rzs[i]]))
        except:
            pass

    t_accs = np.array(t_accs)
    return t_accs[:, 0], t_accs[:, 1], t_accs[:, 2]

def compute_windowed_acc_and_vel(axs, ays, azs, rxs, rys, rzs, rolling, is_50hz):
    """ processes input accelerations by windowing or rolling averages

    takes in accs and rots, applies rots to accs, and computes either rolling or windowed average on 
    transformed accelerations, as well as corresponding velocities

    :rtype: list of rotated base vector
    """
    if is_50hz:
        window_size = 50
    else: 
        window_size = 20
    prev_i = 0
    # correction_x = 0.054
    # correction_y = 0.55
    # correction_z = 0.855
    correction_x = 0
    correction_y = 0
    correction_z = 0

    axs = [x + correction_x for x in axs]
    ays = [x + correction_y for x in ays]
    azs = [x + correction_z for x in azs]

    axs, ays, azs = compute_transformed_accelerations(axs, ays, azs, rxs, rys, rzs)

    windowed_ax = []
    windowed_ay = []
    windowed_az = []

    vxs = []
    vys = []
    vzs = []

    if (rolling):
        # rolling window
        for i in range(0, len(axs) - 1 - window_size):
            windowed_ax.append(np.average(np.array(axs[i:i + window_size])))
            windowed_ay.append(np.average(np.array(ays[i:i + window_size])))
            windowed_az.append(np.average(np.array(azs[i:i + window_size])))
            
            vxs.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in axs[i:i + window_size]])))
            vys.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in ays[i:i + window_size]])))
            vzs.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in azs[i:i + window_size]])))
    else:
        # average window
        for i in range(window_size, len(axs) - 1, window_size):
            windowed_ax.append(np.average(np.array(axs[prev_i:i])))
            windowed_ay.append(np.average(np.array(ays[prev_i:i])))
            windowed_az.append(np.average(np.array(azs[prev_i:i])))
            
            vxs.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in axs[prev_i:i]])))
            vys.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in ays[prev_i:i]])))
            vzs.append(np.sum(np.array([((1.0 / window_size) * x * 9.8) for x in azs[prev_i:i]])))

            prev_i = i

    axs = windowed_ax
    ays = windowed_ay
    azs = windowed_az

    return axs, ays, azs, vxs, vys, vzs

def plot_avr(relative_filename, axs, ays, azs, vxs, vys, vzs, rxs, rys, rzs):
    t1a = np.arange(0, len(axs))
    t2a = np.arange(0, len(ays))
    t3a = np.arange(0, len(azs))

    t1v = np.arange(0, len(vxs))
    t2v = np.arange(0, len(vys))
    t3v = np.arange(0, len(vzs))

    t1r = np.arange(0, len(rxs))
    t2r = np.arange(0, len(rys))
    t3r = np.arange(0, len(rzs))

    fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=3, figsize=(12,8)) # nrows = avr

    ax1[0].plot(t1r, rxs)
    ax1[0].set_title("raw rxs over time")
    ax1[1].plot(t2r, rys)
    ax1[1].set_title("raw rys over time")
    ax1[2].plot(t3r, rzs)
    ax1[2].set_title("raw rzs over time")

    ax2[0].plot(t1a, axs)
    ax2[0].set_title("transformed axs over time")
    ax2[1].plot(t2a, ays)
    ax2[1].set_title("transformed ays over time")
    ax2[2].plot(t3a, azs)
    ax2[2].set_title("transformed azs over time")

    ax3[0].plot(t1v, vxs)
    ax3[0].set_title("transformed vxs over time")
    ax3[1].plot(t2v, vys)
    ax3[1].set_title("transformed vys over time")
    ax3[2].plot(t3v, vzs)
    ax3[2].set_title("transformed vzs over time")

    fig.suptitle(relative_filename)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--single_file', help="The split csv file to use, e.g. random-Sat Nov 21 17_12_50 2020_1.csv", action="store")
    parser.add_argument("--average_window", help="use average window (default is rolling window)", action="store_true")
    parser.add_argument("--show_plots", help="show plots of position curves", action="store_true")
    parser.add_argument("--is_50hz", help="use 50hz data (default is 200hz data)", action="store_true")
    parser.add_argument("--use_csv", help="use csv data (default is split_csv data)", action="store_true")
    
    args = parser.parse_args()

    is_rolling = not args.average_window # rolling window or average window flag
    is_50hz = args.is_50hz # is data sampled at 50hz or 200hz
    use_split_csv = not args.use_csv # use split_csv or csv directory
    show_plots = args.show_plots # show plots or not (if looping through, better not to)

    hz_string = "50hz" if is_50hz else "200hz"
    csv_directory_string = "split_csv" if use_split_csv else "csv"

    hz_directory = os.path.join(get_basepath(), "data", csv_directory_string, hz_string)
    if args.single_file is not None:
        relative_filenames = [args.single_file]
    else:
        relative_filenames = os.listdir(hz_directory)
    for relative_filename in relative_filenames:
        filename = os.path.join(get_basepath(), "data", csv_directory_string, hz_string, relative_filename)

        axs, ays, azs, rxs, rys, rzs = parse_input_file(filename)
        axs, ays, azs, vxs, vys, vzs = compute_windowed_acc_and_vel(axs, ays, azs, rxs, rys, rzs, is_rolling, is_50hz)
        if show_plots:
            plot_avr(relative_filename, axs, ays, azs, vxs, vys, vzs, rxs, rys, rzs)
            # plot3d(
            #     xyzs=[(vxs, vys, vzs)],
            #     labels=["plotted imu velocities"],
            #     title=relative_filename
            # )
            # plot2d(
            #     xys=[(vxs, vys)],
            #     labels=["plotted imu velocities"],
            #     title=relative_filename
            # )

        rolling_word = "rolling_window" if is_rolling else "average_window"
        output_filename = os.path.join(get_basepath(), "data", "imu_vs", relative_filename[:-4] + "_imu_vel_" + rolling_word + ".csv")
        write_output_file(output_filename, vxs, vys, vzs)

