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
            try:
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
            except:
                continue
            try:
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
            except:
                continue

            # if length of sample at least MINLEN and velocity is close to 0, create new file.
            if (np.average(vels) < 0.1 and length > MINLEN):
                nprows = np.array(rows)
                values = nprows[:, CSVIDXMAP["rz"]].astype(float) 

                # update rzs with additive correction
                # AFTER gyro -> rotation is corrected, if we have it
                if correct_rzs:
                    slope = linregress(np.flip(np.arange(len(rzs))), rzs).slope
                    additive_correction = slope * np.flip(np.arange(len(nprows)))
                    nprzs = nprows[:, CSVIDXMAP["rz"]].astype(float) 
                    nprzs += additive_correction
                    nprows[:, CSVIDXMAP["rz"]] = nprzs

                if length < MINLEN * 2 * 6:
                    filenum += 1
                    np.savetxt(outwriter, nprows, delimiter=",", fmt='%s')
                    outwriter.close()
                    outwriter = open(outfile[:-4] + "_" +str(filenum) + ".csv", "w")
                    rows = []
                else:
                    split_nprows = np.array_split(nprows, length // (MINLEN * 6))

                    for nprow in split_nprows:
                        filenum += 1
                        np.savetxt(outwriter, nprow, delimiter=",", fmt='%s')
                        outwriter.close()
                        outwriter = open(outfile[:-4] + "_" +str(filenum) + ".csv", "w")
                        rows = []
                length = 0
            length += 1
            # except Exception as e:
            #     # sometimes the last row is incomplete
            #     print("Error:", str(e))
            #     pass
    os.remove(outwriter.name) # remove last opened csv, which was not written to


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--single_file', help="The csv file to use, e.g. random-Sat Nov 21 17_12_50 2020.csv", action="store")
    parser.add_argument("--is_50hz", help="use 50hz data (default is 200hz data)", action="store_true")
    parser.add_argument("--regular_rzs", help="don't use rz correction (default use rz correction)", action="store_true")
    parser.add_argument("--regular_gyro", help="don't use gyro correction (default use gyro correction)", action="store_true")
    
    args = parser.parse_args()

    is_50_hz = args.is_50hz
    correct_rzs = not args.regular_rzs

    hz_string = "50hz" if is_50_hz else "200hz"
    if args.single_file is not None:
        directory = [args.single_file]
    else:
        directory = os.listdir(os.path.join(get_basepath(), "data", "csv", hz_string))
    for fname in directory:
        if ".csv" not in fname:
            continue
        infile = os.path.join(get_basepath(), "data", "csv", hz_string, fname)
        outfile = os.path.join(get_basepath(), "data", "split_csv", hz_string, fname)
        csv_split(infile, outfile, correct_rzs, 0)
    # else:
    #     print("USAGE: python log_to_gpx.py $INFILE $OUTFILE")
    #     exit()

    