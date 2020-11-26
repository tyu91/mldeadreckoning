from geopy import distance
from geopy.distance import lonlat
import csv
import os
import sys
from collections import defaultdict
import math
from pathlib import Path

basepath = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def calc_velocity(filepath, filename):
    dt = 1 # to be changed later cuz were only sampling at 1Hz
    alt = []
    lat_lon = []
    time = []

    with open(filepath) as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            try:
                z = float(row[11][1:])
                lat = float(row[9][1:])
                lon = float(row[10][1:])
                temp = [lat, lon]

                # unique times only
                if(row[12] not in time):
                    time.append(row[12])
                    #store into lists
                    alt.append(z)
                    lat_lon.append([lat, lon])

            except:
                # sometimes the last row is incomplete
                pass


    # velocity time, assuming sampled at 1hz
    velx = [] #x axis is east west direction
    vely = [] #y axis is north south direction
    vel = []

    for i in range (len(lat_lon)-1):
        ref_lat = lat_lon[i+1][0]
        ref_long = lat_lon[i][1]
        ref_point = [ref_lat, ref_long]
        distx = distance.geodesic(lat_lon[i], ref_point,  ellipsoid='WGS-84').km * 1000
        disty = distance.geodesic(lat_lon[i+1], ref_point,  ellipsoid='WGS-84').km * 1000
        velx.append(distx/dt) #since its 1 second update
        vely.append(disty/dt) #since its 1 second update
        vel.append(distance.geodesic(lat_lon[i+1], lat_lon[i],  ellipsoid='WGS-84').km * 1000)


    # elevation in feet convert to meters
    velz = []
    for i in range (len(alt)-1):
        velz.append((alt[i+1]-alt[i])*0.3048/dt)

    # sanity check seems good
    # for i in range(len(velx)):
        # print(vel[i])
        # print(math.sqrt((velx[i]*velx[i]) + (vely[i]*vely[i])))
        # print("\n")

    # write to csv
    csvname = os.path.join(basepath, "data", "gps_vs", filename[:-4] + "_gps_vel.csv")
    print(csvname)
    velx = velx[1:]
    vely = vely[1:]
    velz = velz[1:]
    with open(csvname, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(velx)):
            writer.writerow([velx[i],vely[i], velz[i]])

if __name__ == "__main__":
    directory_50 = os.path.join(basepath, "data","csv", "50hz")

    for filename in os.listdir(directory_50):
        if(filename.endswith(".csv")):
            filepath = os.path.join(directory_50, filename)
            calc_velocity(filepath, filename)

    directory_200 = os.path.join(basepath, "data","csv", "200hz")
    for filename in os.listdir(directory_200):
        if(filename.endswith(".csv")):
            filepath = os.path.join(directory_200, filename)
            calc_velocity(filepath, filename)
    