from odometry import *
from utils import *

if __name__ == "__main__":
    single_file = True # perform odometry on single file vs. all files
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


