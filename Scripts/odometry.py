import pandas as pd
import csv
import os
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def get_vs_from_file(filename):
    """gets velocities from file
    :rtype: x, y, z velocities as lists
    """
    pass

    print(filename)

    df = pd.read_csv(filename)
    print(df.shape)

    vxs = []
    vys = []
    vzs = []

    with open(filename) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                # TODO: take out abs once gps velocity has direction
                vxs.append((float(row[0])))
                vys.append((float(row[1])))
                vzs.append((float(row[2])))
            except:
                # sometimes the last row is incomplete
                pass

    return vxs, vys, vzs

def get_xyz_poses(vxs, vys, vzs, dt):
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

    return pos_x, pos_y, pos_z

if __name__ == "__main__":
    single_file = False 
    # dt = 0.5 #1Hz
    imu_dt = 1.0 / 50 # 50Hz
    gps_dt = 1 # 1Hz

    basepath = sys.path[0][:-7]
    imu_vs_directory = os.path.join(basepath, "data", "imu_vs")
    gps_vs_directory = os.path.join(basepath, "data", "gps_vs")

    gps_vs_string = "_gps_vel.csv"

    if single_file:
        imu_paths = ["Sun Nov 15 17_52_12 2020_imu_vel_rolling_window.csv"]
    else:
        imu_paths = os.listdir(imu_vs_directory)

    # imu_file = os.path.join(sys.path[0][:-7], "data", "imu_vs", "Sun Nov 15 17_52_12 2020_imu_vel_rolling_window.csv")
    # gps_file = os.path.join(sys.path[0][:-7], "data", "gps_vs", "Sun Nov 15 17_52_12 2020_gps_vel.csv")

    for imu_relative_path in imu_paths:
        if "csv" not in imu_relative_path:
            continue
        print(imu_relative_path)
        imu_file = os.path.join(imu_vs_directory, imu_relative_path)
        gps_file = os.path.join(gps_vs_directory, imu_relative_path.split("_imu_")[0] + gps_vs_string)
    
        imu_vxs, imu_vys, imu_vzs = get_vs_from_file(imu_file)
        imu_pxs, imu_pys, imu_pzs = get_xyz_poses(imu_vxs, imu_vys, imu_vzs, imu_dt)

        gps_vxs, gps_vys, gps_vzs = get_vs_from_file(gps_file)
        gps_pxs, gps_pys, gps_pzs = get_xyz_poses(gps_vxs, gps_vys, gps_vzs, gps_dt)
        

        imu_t = np.arange(0, len(imu_pxs))
        gps_t = np.arange(0, len(gps_pxs))

        mpl.rcParams['legend.fontsize'] = 10
        fig = plt.figure()
        # ax = fig.gca(projection='3d')
        plt.plot(imu_pxs, imu_pys,label='positions from imu velocity curve')
        # plt.plot(gps_pxs, gps_pys,label='positions from gps velocity curve')
        plt.legend()
        plt.show()


