import gpxpy
import gpxpy.gpx
import csv
import sys
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress


from utils import *

# minimum split length of 10 seconds, avoids a bunch of garbage at a long light
MINLEN = 200 * 10
def is_float(num):
    try:
        float(num)
        return True
    except:
        return False

def csv_split(infile, outfile, correct_rzs):
    vels = []
    rzs = []
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

                vels.append(float(row[CSVIDXMAP["velocity"]]))
                vels = vels[-400:]
                rzs.append(float(row[CSVIDXMAP["rz"]]))
                rzs = rzs[-400:]

                # if length of sample at least MINLEN and velocity is close to 0, create new file.
                if (np.average(vels) < 0.1 and length > MINLEN):
                    nprows = np.array(rows)
                    values = nprows[:, CSVIDXMAP["rz"]].astype(float) 

                    # update rzs with additive correction
                    if correct_rzs:
                        slope = linregress(np.flip(np.arange(len(rzs))), rzs).slope
                        additive_correction = slope * np.flip(np.arange(len(nprows)))
                        nprzs = nprows[:, CSVIDXMAP["rz"]].astype(float) 
                        nprzs += additive_correction
                        nprows[:, CSVIDXMAP["rz"]] = nprzs

                    # do file shit
                    length = 0
                    filenum += 1
                    np.savetxt(outwriter, nprows, delimiter=",", fmt='%s')
                    outwriter.close()
                    outwriter = open(outfile[:-4] + "_" +str(filenum) + ".csv", "w")
                    rows = []
                length += 1
            except:
                # sometimes the last row is incomplete
                pass
    os.remove(outwriter.name) # remove last opened csv, which was not written to


if __name__ == "__main__":

    # csv_split("../data/csv/random-Sat Nov 21 16_20_53 2020.csv")
    is_50_hz = False
    correct_rzs = True
    single_file = True

    hz_string = "50hz" if is_50_hz else "200hz"
    if len(sys.argv) == 3:
        csv_split(sys.argv[1], sys.argv[2])
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

    