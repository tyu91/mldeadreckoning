import gpxpy
import gpxpy.gpx
import csv
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress
from ahrs.common import orientation
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

from utils import *

# minimum split length of 10 seconds, avoids a bunch of garbage at a long light
MINLEN = 200 * 10
def is_float(num):
    try:
        float(num)
        return True
    except:
        return False

def csv_split(infile, outfile, correct_rzs, correct_gyro):
    vels = []
    rzs = []
    gxs = []
    gys = []
    gzs = []
    rows = []
    filenum = 0
    length = 0
    outwriter = open(outfile[:-4] + "_" +str(filenum) + ".csv", "w")
    ctr = 0
    with open(infile) as csvfile:
        reader = csv.reader(csvfile)
        new_file = True
        for row in reader:
            # try:
            if not is_float(row[0]): # if it's the header, drop it
                continue
            rows.append(row)
            gxs.append(float(row[CSVIDXMAP["gx"]]))
            gxs = gxs[-400:]
            gys.append(float(row[CSVIDXMAP["gy"]]))
            gys = gys[-400:]
            gzs.append(float(row[CSVIDXMAP["gz"]]))
            gzs = gzs[-400:]
            vels.append(float(row[CSVIDXMAP["velocity"]]))
            vels = vels[-400:]
            rzs.append(float(row[CSVIDXMAP["rz"]]))
            rzs = rzs[-400:]

            # if length of sample at least MINLEN and velocity is close to 0, create new file.
            if (np.average(vels) < 0.1 and length > MINLEN):
                nprows = np.array(rows)
                values = nprows[:, CSVIDXMAP["rz"]].astype(float) 

                

                # calculate gyro offsets
                if correct_gyro:
                    # under the assumption that we have now been stationary, or near it, for a while
                    # gx_adj = -np.average(gxs)
                    # gy_adj = -np.average(gys)
                    # gz_adj = -np.average(gzs)
                    # npgxs = nprows[:, CSVIDXMAP["gx"]].astype(float) 
                    # npgxs = np.add(npgxs, gx_adj)
                    # nprows[:, CSVIDXMAP["gx"]] = npgxs
                    # npgys = nprows[:, CSVIDXMAP["gy"]].astype(float) 
                    # npgys = np.add(npgys, gy_adj)
                    # nprows[:, CSVIDXMAP["gy"]] = npgys
                    # npgzs = nprows[:, CSVIDXMAP["gz"]].astype(float) 
                    # npgzs = np.add(npgzs, gz_adj)
                    # nprows[:, CSVIDXMAP["gz"]] = npgzs

                    # print("GX:", gx_adj)
                    # print("GY:", gy_adj)
                    # print("GZ:", gz_adj)

                    # recalc all the sensor fusion...
                    acc_data = np.array([nprows[:, CSVIDXMAP["ax"]].astype(float), nprows[:, CSVIDXMAP["ay"]].astype(float), nprows[:, CSVIDXMAP["az"]].astype(float)]).transpose()
                    gyro_data = np.array([nprows[:, CSVIDXMAP["gx"]].astype(float), nprows[:, CSVIDXMAP["gy"]].astype(float), nprows[:, CSVIDXMAP["gz"]].astype(float)]).transpose()

                    print(gyro_data[0])
                    init = [nprows[:, CSVIDXMAP["rx"]].astype(float)[0], nprows[:, CSVIDXMAP["ry"]].astype(float)[0], nprows[:, CSVIDXMAP["rz"]].astype(float)[0]]
                    # get initial orientation from the data we already have
                    r = R.from_euler(seq='xyz', angles=init, degrees=True)
                    q_last = r.as_quat()
                    madgwick = Madgwick(frequency=200.0, q0 = r.as_quat(), gain=0.4)
                    print("init:")
                    eulers = r.as_euler(seq='xyz', degrees=True)
                    print("x:", eulers[0], "y:", eulers[1], "z:", eulers[2])
                    new_rxs = []
                    new_rys = []
                    new_rzs = []
                    for i in range(len(acc_data)):
                        conv_gyro = (gyro_data[i])
                        conv_acc = (acc_data[i] * 9.81)
                        print("acc:", conv_acc)
                        print("gyr:", conv_gyro)
                        q = madgwick.updateIMU(gyr=conv_gyro, acc=conv_acc, q=q_last)
                        eulers = R.from_quat(q).as_euler(seq='xyz', degrees=True)
                        print("x:", eulers[0], "y:", eulers[1], "z:", eulers[2])
                        new_rxs.append(eulers[0])
                        new_rys.append(eulers[1])
                        new_rzs.append(eulers[2])
                        q_last = q

                    nprows[:, CSVIDXMAP["rx"]] = new_rxs
                    nprows[:, CSVIDXMAP["ry"]] = new_rys
                    nprows[:, CSVIDXMAP["rz"]] = new_rzs
                    # quats = madgwick.Q.shape


                # update rzs with additive correction
                # AFTER gyro -> rotation is corrected, if we have it
                if correct_rzs:
                    slope = linregress(np.flip(np.arange(len(rzs))), rzs).slope
                    additive_correction = slope * np.flip(np.arange(len(nprows)))
                    nprzs = nprows[:, CSVIDXMAP["rz"]].astype(float) 
                    nprzs += additive_correction
                    nprows[:, CSVIDXMAP["rz"]] = nprzs

                # do file shit
                length = 0
                print(filenum)
                filenum += 1
                np.savetxt(outwriter, nprows, delimiter=",", fmt='%s')
                outwriter.close()
                outwriter = open(outfile[:-4] + "_" +str(filenum) + ".csv", "w")
                rows = []
            length += 1
            # except Exception as e:
            #     # sometimes the last row is incomplete
            #     print("Error:", str(e))
            #     pass
    os.remove(outwriter.name) # remove last opened csv, which was not written to


if __name__ == "__main__":

    # csv_split("../data/csv/random-Sat Nov 21 16_20_53 2020.csv")
    is_50_hz = False
    correct_rzs = True
    single_file = True

    hz_string = "50hz" if is_50_hz else "200hz"
    if len(sys.argv) == 3:
        csv_split(sys.argv[1], sys.argv[2], False, True)
    if len(sys.argv) == 1:
        if single_file:
            directory = ["random-Sat Nov 21 17_12_50 2020.csv"]
        else:
            directory = os.listdir(os.path.join(get_basepath(), "data", "csv", hz_string))
        for fname in directory:
            if ".csv" not in fname:
                continue
            infile = os.path.join(get_basepath(), "data", "csv", hz_string, fname)
            outfile = os.path.join(get_basepath(), "data", "split_csv", hz_string, fname)
            csv_split(infile, outfile, correct_rzs)
    # else:
    #     print("USAGE: python log_to_gpx.py $INFILE $OUTFILE")
    #     exit()

    