import pandas as pd
import csv
import os
import sys
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import json
import utm
import math

from utils import *

def get_xyz_poses_from_arr(vxs, vys, vzs, dt):
    pos_x = []
    pos_y = []
    pos_z = []

    #initial pose
    pos_x.append(0)
    pos_y.append(0)
    pos_z.append(0)
    prev_posx = 0
    prev_posy = 0
    prev_posz = 0
    for i in range(len(vxs)-1):
        prev_vx = vxs[i]
        prev_vy = vys[i]
        prev_vz = vzs[i]
        curr_vx = vxs[i+1]
        curr_vy = vys[i+1]
        curr_vz = vzs[i+1]
        if (math.isclose(prev_vx, curr_vx) and math.isclose(prev_vy, curr_vy) and math.isclose(prev_vz, curr_vz)):
            pos_x.append(prev_posx)
            pos_y.append(prev_posy)
            pos_z.append(prev_posz)
        else:
            print(i)
            prev_posx = pos_x[i] + vxs[i+1]*dt
            prev_posy = pos_y[i] + vys[i+1]*dt
            prev_posz = pos_z[i] + vzs[i+1]*dt
            pos_x.append(pos_x[i] + vxs[i+1]*dt)
            pos_y.append(pos_y[i] + vys[i+1]*dt)
            pos_z.append(pos_z[i] + vzs[i+1]*dt)

    return pos_x, pos_y, pos_z

def get_vs_from_file(filename):
    """gets velocities from file

    :rtype: x, y, z velocities as lists
    """

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

def get_ps_from_file(filename):
    """gets positions from og gps file

    :rtype: x, y, z positions as lists
    """
    print(filename)

    df = pd.read_csv(filename)
    print(df.shape)

    pxs = []
    pys = []

    started = False

    offsetx = 0
    offsety = 0

    with open(filename) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                lat = float(row[9][1:])
                lon = float(row[10][1:])
                if lat == 0 or lon == 0:
                    continue
                x, y, _, _ = utm.from_latlon(lat, lon)
                if not started:
                    offsetx = x
                    offsety = y
                    started = True
                pxs.append(x - offsetx)
                pys.append(y - offsety)

            except:
                # sometimes the last row is incomplete
                pass

    return pxs, pys

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

def compute_imu_gps_xyz_poses(base_filename, imu_file, gps_file, og_gps_directory, imu_dt, gps_dt):
    imu_vxs, imu_vys, imu_vzs = get_vs_from_file(imu_file)
    gps_vxs, gps_vys, gps_vzs = get_vs_from_file(gps_file)
    actual_gps_pxs, actual_gps_pys = get_ps_from_file(os.path.join(og_gps_directory, base_filename + ".csv"))

    imu_pxs, imu_pys, imu_pzs = get_xyz_poses(imu_vxs, imu_vys, imu_vzs, imu_dt)
    gps_pxs, gps_pys, gps_pzs = get_xyz_poses(gps_vxs, gps_vys, gps_vzs, gps_dt)
    
    return imu_pxs, imu_pys, imu_pzs, gps_pxs, gps_pys, gps_pzs

if __name__ == "__main__":
    single_file = False # perform odometry on single file vs. all files
    tag_files = False # write tags to file
    show_plots = True # plot positions
    is_50hz = False 

    imu_dt = 1.0 / 50 if is_50hz else 1.0 / 200
    gps_dt = 1 if is_50hz else 1

    hz_string = "50hz" if is_50hz else "200hz"

    imu_vs_directory = os.path.join(get_basepath(), "data", "imu_vs")
    gps_vs_directory = os.path.join(get_basepath(), "data", "gps_vs")
    og_gps_directory = os.path.join(get_basepath(), "data", "split_csv", hz_string)

    gps_vs_string = "_gps_vel.csv"

    if single_file:
        imu_paths = ["random-Sat Nov 21 17_12_50 2020_1_imu_vel_rolling_window.csv"]
    else:
        imu_paths = os.listdir(imu_vs_directory)
    
    tag_dict = {} # only used of tag_files set to True

    for imu_relative_path in imu_paths:
        if "csv" not in imu_relative_path:
            continue
        # if "stationary" in imu_relative_path or "random" in imu_relative_path or "boosting" in imu_relative_path:
        #   continue
        imu_file = os.path.join(imu_vs_directory, imu_relative_path)
        base_filename = imu_relative_path.split("_imu_")[0]
        gps_file = os.path.join(gps_vs_directory, base_filename + gps_vs_string)

        imu_pxs, imu_pys, imu_pzs, gps_pxs, gps_pys, gps_pzs = compute_imu_gps_xyz_poses(
                                                                                         base_filename, 
                                                                                         imu_file, gps_file, 
                                                                                         og_gps_directory, 
                                                                                         imu_dt, 
                                                                                         gps_dt
                                                                                        )

        imu_t = np.arange(0, len(imu_pxs))
        gps_t = np.arange(0, len(gps_pxs))

        if show_plots:
            # plot3d(
            #         xyzs=[(imu_pxs, imu_pys, imu_pzs), (gps_pxs, gps_pys, gps_pzs)],
            #         labels=["positions from imu velocity curve", "positions from gps velocity curve"],
            #         title=base_filename
            #     )
            
            plot2d(
                    xys=[(imu_pxs, imu_pys), (gps_pxs, gps_pys)],
                    labels=["positions from imu velocity curve", "positions from gps velocity curve"],
                    title=base_filename
                )

        # if in tag files mode
        if tag_files:
            description = input("Please enter a description for this file\n")
            if len(description) > 0:
                gpx_filename = base_filename + ".gpx"
                print(gpx_filename)
                tag_dict[gpx_filename] = description
    if tag_files:
        with open(os.path.join(basepath, "resources", "transformation_hope.json"), "w") as jsonfile:
            json.dump(tag_dict, jsonfile, indent=4)


