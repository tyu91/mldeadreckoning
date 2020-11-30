import gpxpy
import gpxpy.gpx
import csv
import sys
import os

def convert_to_gpx(infile, outfile):
    lats = []
    longs = []
    alts = []
    with open(infile) as csvfile:
        reader = csv.reader(csvfile)
        
        for row in reader:
            try:
                alts.append(row[11])
                longs.append(row[10])
                lats.append(row[9])
                
            except:
                # sometimes the last row is incomplete
                pass
    
    gpx = gpxpy.gpx.GPX()

    # Create first track in our GPX:
    gpx_track = gpxpy.gpx.GPXTrack()
    gpx.tracks.append(gpx_track)

    # Create first segment in our GPX track:
    gpx_segment = gpxpy.gpx.GPXTrackSegment()
    gpx_track.segments.append(gpx_segment)

    # Create points:
    for i in range(len(lats)): # poor practice, sue me
        gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lats[i], longs[i], elevation=alts[i]))

    with open(outfile, "w") as gpxfile:
        gpxfile.write(gpx.to_xml())


if __name__ == "__main__":
    is_50_hz = True
    hz_string = "50hz" if is_50_hz else "200hz"
    if len(sys.argv) == 3:
        convert_to_gpx(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 1:
        for fname in os.listdir(os.path.join(sys.path[0][:-7], "data", "csv", hz_string)):
            infile = os.path.join(sys.path[0][:-7], "data", "csv", hz_string, fname)
            outfile = os.path.join(sys.path[0][:-7], "data", "gpx", fname.replace(".csv", ".gpx"))
            convert_to_gpx(infile, outfile)
    else:
        print("USAGE: python log_to_gpx.py $INFILE $OUTFILE")
        exit()

    