import gpxpy
import gpxpy.gpx
import csv
import sys
import os
import math
import numpy as np


from utils import *

# minimum split length of 10 seconds, avoids a bunch of garbage at a long light
MINLEN = 200 * 10

def csv_split(infile, outfile):
    vels = []
    file = 0
    length = 0
    outWriter = open(outfile[:-4] + "_" +str(file) + ".csv", "w")
    with open(infile) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                outWriter.write(",".join(row) + "\n")
                vels.append(float(row[13]))
                vels = vels[-400:]
                if (np.average(vels) < 0.1 and length > MINLEN):
                    length = 0
                    file = file + 1
                    outWriter.close()
                    outWriter = open(outfile[:-4] + "_" +str(file) + ".csv", "w")

                length = length + 1

            except:
                # sometimes the last row is incomplete
                pass                

if __name__ == "__main__":

    # csv_split("../data/csv/random-Sat Nov 21 16_20_53 2020.csv")
    is_50_hz = False

    hz_string = "50hz" if is_50_hz else "200hz"
    if len(sys.argv) == 3:
        csv_split(sys.argv[1], sys.argv[2])
    if len(sys.argv) == 1:
        for fname in os.listdir(os.path.join(get_basepath(), "data", "csv", hz_string)):
            if ".csv" not in fname:
                continue
            infile = os.path.join(get_basepath(), "data", "csv", hz_string, fname)
            outfile = os.path.join(get_basepath(), "data", "split_csv", hz_string, fname)
            csv_split(infile, outfile)
    # else:
    #     print("USAGE: python log_to_gpx.py $INFILE $OUTFILE")
    #     exit()

    