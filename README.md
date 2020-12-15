# ML Dead Reckoning Project

Ensure you have the following data file structure before running:

### data

* [csv/](./data/csv)
  * [200hz/](./data/csv/200hz)
  * [50hz/](./data/csv/50hz)
* [gps_vs/](./data/gps_vs)
* [gpx/](./data/gpx)
* [imu_vs/](./data/imu_vs)
* [split_csv/](./data/split_csv)
  * [200hz/](./data/split_csv/200hz)
  * [50hz/](./data/split_csv/50hz)

To generate the necessary files:
1. download all the gps files and place into the `csv/200hz` directory. 
2. Ensure the `single_file = False`, `is_50hz = False`, `show_plots = False`, `use_split_csv = True` flags are set appropriately in the following files:
    * `split_csv.py`
    * `process_data.py`
    * `gps_to_vel.py`

3. Run `python split_csv.py && python process_data.py && python gps_to_vel.py` from the `Scripts/` directory to generate the proper files (this will take a while, like 15-30 minutes). 

4. (Optional) To generate the gpx files, run `python log_to_gpx.py` from the `Scripts/` directory. 

5. (Optional) In order to view imu and velocity positions, run `python odometry.py` from the `Scripts/` directory with `show_plots = True` flag.
