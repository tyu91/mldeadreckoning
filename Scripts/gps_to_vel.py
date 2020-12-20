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

def calc_velocity_from_arr(input_arr):
    dt = 1 # to be changed later cuz were only sampling at 1Hz
    alt = []
    lat_lon = []
    time = []

    for row in input_arr:
        try:
            z = float(row[CSVIDXMAP["alt"]][1:])
            lat = float(row[CSVIDXMAP["lat"]][1:])
            lon = float(row[CSVIDXMAP["long"]][1:])
            temp = [lat, lon]

            # unique times only
            #if(row[12] not in time):
            time.append(row[CSVIDXMAP["time"]])
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
        prev_lat = lat_lon[i][0]
        prev_lon = lat_lon[i][1]
        curr_lat = lat_lon[i + 1][0]
        curr_lon = lat_lon[i + 1][1]
        if prev_lat == 0 or prev_lon == 0:
            continue
        if (math.isclose(prev_lat, curr_lat) and math.isclose(prev_lon, curr_lon)):
            if i==0:
                velx.append(0)
                vely.append(0)
                vel.append(0)
            else:
                velx.append(velx[i-1])
                vely.append(vely[i-1])
                vel.append(vel[i-1])
            continue
        prev_x, prev_y, _, _ = utm.from_latlon(prev_lat, prev_lon)
        curr_x, curr_y, _, _ = utm.from_latlon(curr_lat, curr_lon)
        
        dirx = 1
        diry = 1

        # direrction
        distx = (curr_x - prev_x) # TODO: negate to display properly but may fuck with position
        disty = curr_y - prev_y
        #print("here")
        velx.append(dirx*distx/dt) #since its 1 second update
        vely.append(diry*disty/dt) #since its 1 second update
        vel.append(distance.geodesic(lat_lon[i+1], lat_lon[i],  ellipsoid='WGS-84').km * 1000)
        
    # elevation in feet convert to meters
    velz = []
    velz.append(0)
    prev_velz = 0
    for i in range (len(alt)-1):
        prev_alt = alt[i]
        curr_alt = alt[i+1]
        if math.isclose(prev_alt, curr_alt):
            velz.append(prev_velz)
        else:
            dirz = 1 if (alt[i] <= alt[i+1]) else -1
            #print((alt[i+1]-alt[i])*0.3048/dt*dirz)
            prev_velz = (alt[i+1]-alt[i])*0.3048/dt*dirz
            velz.append((alt[i+1]-alt[i])*0.3048/dt*dirz)

    velx.append(velx[-1])
    vely.append(vely[-1])
    velz.append(velz[-1])
    velz = velz[1:]
    return [velx, vely, velz]


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

    # offsetx = 0
    # offsety = 0
    # started_path = False
    
    for i in range (len(lat_lon)-1):
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
        distx = (curr_x - prev_x) # TODO: negate to display properly but may fuck with position
        disty = curr_y - prev_y
        
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
    parser = argparse.ArgumentParser()

    parser.add_argument('--single_file', help="The split csv file to use, e.g. random-Sat Nov 21 17_12_50 2020_1.csv", action="store")
    parser.add_argument("--is_50hz", help="use 50hz data (default is 200hz data)", action="store_true")
    parser.add_argument("--use_csv", help="use csv data (default is split_csv data)", action="store_true")
    
    args = parser.parse_args()

    is_50hz = args.is_50hz
    use_split_csv = not args.use_csv

    hz_string = "50hz" if is_50hz else "200hz"
    csv_directory_string = "split_csv" if use_split_csv else "csv"

    directory = os.path.join(get_basepath(), "data",csv_directory_string, hz_string)
    directory_list = os.listdir(directory)
    if args.single_file is not None:
        directory_list = [args.single_file]
    for filename in directory_list:
        if(filename.endswith(".csv")):
            filepath = os.path.join(directory, filename)
            calc_velocity(filepath, filename)
    