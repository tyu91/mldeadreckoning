from geopy import distance
from geopy.distance import lonlat
import csv
import os
import sys
from collections import defaultdict
import math
from pathlib import Path
import utm
from utils import *


def calc_velocity(filepath, filename, from_utm=False):
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

    # offsetx = 0
    # offsety = 0
    # started_path = False
    
    for i in range (len(lat_lon)-1):
        if from_utm:
            prev_lat = lat_lon[i][0]
            prev_lon = lat_lon[i][1]
            curr_lat = lat_lon[i + 1][0]
            curr_lon = lat_lon[i + 1][1]
            if prev_lat == 0 or prev_lon == 0:
                continue
            prev_x, prev_y, _, _ = utm.from_latlon(prev_lat, prev_lon)
            curr_x, curr_y, _, _ = utm.from_latlon(curr_lat, curr_lon)
            
            dirx = 1
            diry = 1

            # direrction
            distx = curr_x - prev_x
            disty = curr_y - prev_y
            velx.append(dirx*distx/dt) #since its 1 second update
            vely.append(diry*disty/dt) #since its 1 second update
            vel.append(distance.geodesic(lat_lon[i+1], lat_lon[i],  ellipsoid='WGS-84').km * 1000)
        else:
            ref_lat = lat_lon[i][0]
            ref_long = lat_lon[i+1][1]
            ref_point = [ref_lat, ref_long]
            
            dirx = 1 if (ref_lat <= lat_lon[i+1][0]) else -1
            diry = 1 if (ref_long >= lat_lon[i][1]) else -1

            # direrction
            distx = distance.geodesic(lat_lon[i], ref_point,  ellipsoid='WGS-84').km * 1000
            disty = distance.geodesic(lat_lon[i+1], ref_point,  ellipsoid='WGS-84').km * 1000
            velx.append(dirx*distx/dt) #since its 1 second update
            vely.append(diry*disty/dt) #since its 1 second update
            vel.append(distance.geodesic(lat_lon[i+1], lat_lon[i],  ellipsoid='WGS-84').km * 1000)

    # elevation in feet convert to meters
    velz = []
    for i in range (len(alt)-1):
        dirz = 1 if (alt[i] <= alt[i+1]) else -1
        velz.append((alt[i+1]-alt[i])*0.3048/dt*dirz)


    # sanity check seems good
    # for i in range(len(velx)):
        # print(vel[i])
        # print(math.sqrt((velx[i]*velx[i]) + (vely[i]*vely[i])))
        # print("\n")

    # write to csv
    csvname = os.path.join(get_basepath(), "data", "gps_vs", filename[:-4] + "_gps_vel.csv")
    print(csvname)
    velx = velx[1:]
    vely = vely[1:]
    velz = velz[1:]
    with open(csvname, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        for i in range(len(velx)):
            writer.writerow([velx[i],vely[i], velz[i]])

if __name__ == "__main__":
    from_utm = True # use utm in velocity conversion, it's what works right now
    is_50hz = False
    use_split_csv = True # use split_csv or csv directory
    single_file = False # convert for single file or all files in hz directory

    hz_string = "50hz" if is_50hz else "200hz"
    csv_directory_string = "split_csv" if use_split_csv else "csv"

    directory = os.path.join(get_basepath(), "data",csv_directory_string, hz_string)
    directory_list = os.listdir(directory)
    if single_file:
        directory_list = ["randomSat Dec  5 17_23_01 2020_2.csv"]
    for filename in directory_list:
        if(filename.endswith(".csv")):
            filepath = os.path.join(directory, filename)
            calc_velocity(filepath, filename, from_utm)
    