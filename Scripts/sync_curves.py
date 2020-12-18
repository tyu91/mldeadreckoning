import scipy.interpolate as interp
import random

from odometry import *
from utils import *

def interpolate_xyz(xs, ys, zs, new_size):
    '''interpolates input xs, ys, zs to new_size
    https://stackoverflow.com/questions/60955197/interpolate-to-larger-1d-array-using-python
    '''
    common_length = np.linspace(0, 1, new_size)
    new_xs = interp.interp1d(np.linspace(0, 1, len(xs)),xs)
    new_ys = interp.interp1d(np.linspace(0, 1, len(ys)),ys)
    new_zs = interp.interp1d(np.linspace(0, 1, len(zs)),zs)

    return new_xs(common_length), new_ys(common_length), new_zs(common_length)

def compute_centered(imu_values, gps_values, imu_cogs, gps_cogs):
    '''compute centered imu and gps values
    '''
    imu_values[:, 0] -= imu_cogs[0]
    imu_values[:, 1] -= imu_cogs[1]

    gps_values[:, 0] -= gps_cogs[0]
    gps_values[:, 1] -= gps_cogs[1]

    return imu_values, gps_values

def compute_homography(imu_values, gps_values):
    '''compute transformation from imu to gps xys for small sample
    '''
    # imu_values = np.stack([imu_pxs, imu_pys], axis=1)
    # gps_values = np.stack([gps_pxs, gps_pys], axis=1)

    mult_values = np.multiply(imu_values, gps_values)
    cross_mult_values = np.multiply(np.flip(imu_values, axis=1), gps_values)

    fst_mult_values = np.vstack([mult_values[:, 0], cross_mult_values[:, 0], gps_values[:, 0]]).T
    snd_mult_values = np.vstack([mult_values[:, 1], cross_mult_values[:, 1], gps_values[:, 1]]).T
    # T = np.dot(Vt.T, U)
    # transformed_imu_values = np.dot(T, centered_imu_values.T).T

    num_elems = imu_values.shape[0]
    zeros_7 = np.zeros((num_elems, 7))
    imu_fst = np.hstack([imu_values, zeros_7])
    imu_fst[:, 2] = 1
    imu_fst = -imu_fst
    imu_fst[:, 6:] = fst_mult_values

    zeros_3 = np.zeros((num_elems, 3))
    zeros_4 = np.zeros((num_elems, 4))
    imu_snd = np.hstack([zeros_3, imu_values, zeros_4])
    imu_snd[:, 5] = 1
    imu_snd = -imu_snd
    imu_snd[:, 6:] = snd_mult_values

    interleaved_imu = np.zeros((2 * num_elems, 9))
    interleaved_imu[0::2] = imu_fst
    interleaved_imu[1::2] = imu_snd

    U, S, Vt = np.linalg.svd(interleaved_imu)

    normalized_transform = np.reshape(Vt[-1, :] / Vt[-1, -1], (3, 3))

    return normalized_transform

def num_inliers(transform, imu_values, gps_values):
    '''compute number of elements having distance within threshold

    Computes distance D = d(imu_values, transform * gps_values) + d(gps_values, inv(transform) * imu_values).
    if D < threshold, then it is an inlier.
    Return the number of samples that are inliers.

    :param transform: normalized transformation matrix
    :param imu_values: full imu xy values (num_elem, 2)
    :param gps_values: full gps xy values (num_elem, 2)
    '''
    num_elems = imu_values.shape[0]
    threshold = 5

    try:
        transform_inv = np.linalg.inv(transform)
    except:
        # if cannot compute inverse, ignore this transform
        return 0, None

    # compute error between gps_values and transform * imu_values
    transform1 = np.dot(transform, np.hstack([imu_values, np.ones((num_elems, 1))]).T).T[:, :2]
    forward_l2_error = np.linalg.norm(transform1 - gps_values, axis=1)

    # compute error between imu_values and inv(transform) * gps_values
    transform2 = np.dot(transform_inv, np.hstack([gps_values, np.ones((num_elems, 1))]).T).T[:, :2]
    backward_l2_error = np.linalg.norm(transform2 - imu_values, axis=1)

    total_l2_error = forward_l2_error + backward_l2_error

    within_threshold = np.count_nonzero(total_l2_error < threshold)

    return within_threshold, total_l2_error

def ransac(imu_values, gps_values):
    '''implementation of RANSAC algorithm to pick best fitting transformation matrix

    :rtype: best-performing transformation
    '''
    num_elems = imu_values.shape[0]
    num_trials = num_elems
    num_samples = 4
    inliers = 0
    best_transform = np.zeros((3, 3))
    best_std = 1000

    for i in range(num_trials):
        indices = random.sample(range(num_elems), num_samples)
        sampled_imu_values = imu_values[indices, :]
        sampled_gps_values = gps_values[indices, :]

        # compute transform based on sample points
        transform = compute_homography(sampled_imu_values, sampled_gps_values)

        # count number of inliers using this transform
        curr_inliers_count, total_l2_error = num_inliers(transform, imu_values, gps_values)
        if total_l2_error is None:
            continue
        curr_std = np.std(total_l2_error)
        if curr_inliers_count > inliers and curr_std < best_std:
            inliers = curr_inliers_count
            best_transform = transform
            best_std = curr_std

    # output transform that results in the largest number of inliers
    return best_transform

        

def compute_affine_transformation(imu_pxs, imu_pys, imu_pzs, gps_pxs, gps_pys, gps_pzs):
    ''' transform imu to gps coordinates
    '''
    # convert to np array for ease of use
    imu_pxs = np.squeeze(np.array(imu_pxs))
    imu_pys = np.squeeze(np.array(imu_pys))
    gps_pxs = np.squeeze(np.array(gps_pxs))
    gps_pys = np.squeeze(np.array(gps_pys))

    imu_values = np.stack([imu_pxs, imu_pys], axis=1)
    gps_values = np.stack([gps_pxs, gps_pys], axis=1)

    num_elems = imu_values.shape[0]

    # compute best transform using ransac algorithm
    best_transform = ransac(imu_values, gps_values)

    transformed_imu_values = np.dot(best_transform, np.hstack([imu_values, np.ones((num_elems, 1))]).T).T

    return transformed_imu_values[:, 0], transformed_imu_values[:, 1], \
        gps_values[:, 0], gps_values[:, 1]

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--single_file', help="The imu velocity file to use, e.g. random-Sat Nov 21 17_12_50 2020_1_imu_vel_rolling_window.csv", action="store")
    parser.add_argument("--tag_files", help="label files and store labels in json", action="store_true")
    parser.add_argument("--show_plots", help="show plots of position curves", action="store_true")
    parser.add_argument("--save_plots", help="save plots to data/figures directory (WILL overwrite existing figures)", action="store_true")
    parser.add_argument("--is_50hz", help="use 50hz data (default is 200hz data)", action="store_true")
    parser.add_argument("--sanity_check", help="ensure that RANSAC algorithm works properly", action="store_true")
    
    args = parser.parse_args()

    tag_files = args.tag_files
    show_plots = args.show_plots
    save_plots = args.save_plots
    is_50hz = args.is_50hz
    sanity_check = args.sanity_check

    imu_dt = 1.0 / 50 if is_50hz else 1.0 / 200
    gps_dt = 1 if is_50hz else 1

    hz_string = "50hz" if is_50hz else "200hz"

    imu_vs_directory = os.path.join(get_basepath(), "data", "imu_vs")
    gps_vs_directory = os.path.join(get_basepath(), "data", "gps_vs")
    og_gps_directory = os.path.join(get_basepath(), "data", "split_csv", hz_string)

    gps_vs_string = "_gps_vel.csv"
    if args.single_file is not None:
        imu_paths = [args.single_file]
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
        interp_factor = 20
        imu_pxs, imu_pys, imu_pzs = interpolate_xyz(imu_pxs, imu_pys, imu_pzs, len(gps_pxs) * interp_factor)
        gps_pxs, gps_pys, gps_pzs = interpolate_xyz(gps_pxs, gps_pys, gps_pzs, len(gps_pxs) * interp_factor)
        if sanity_check:
            fake_transform = np.array([[0.5, -0.5, 3], [3, 1, 7], [0, 0, 1]])
            fake_imu =  np.random.rand(100, 2)  
            fake_gps = np.dot(fake_transform, np.hstack([fake_imu, np.ones((100, 1))]).T).T[:, :2]                                                                                                                                                
            imu_pxs, imu_pys, gps_pxs, gps_pys = compute_affine_transformation(fake_imu[:, 0], fake_imu[:, 1], None, [fake_gps[:, 0]], [fake_gps[:, 1]], None)
        else:
            new_imu_pxs, new_imu_pys, new_gps_pxs, new_gps_pys = compute_affine_transformation(imu_pxs, imu_pys, imu_pzs, gps_pxs, gps_pys, gps_pzs)

        assert(len(new_imu_pxs) == len(new_gps_pxs))

        imu_t_new = np.arange(0, len(new_imu_pxs))
        gps_t_new = np.arange(0, len(new_gps_pxs))

        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, figsize=(12,8)) # nrows = avr

        ax1.plot(new_imu_pxs, new_imu_pys)
        ax1.plot(new_gps_pxs, new_gps_pys)
        # ax1.xlabel("m")
        # ax1.ylabel("m")
        ax1.set_title("with transformation")

        ax2.plot(imu_pxs, imu_pys)
        ax2.plot(gps_pxs, gps_pys)
        # ax2.xlabel("m")
        # ax2.ylabel("m")
        ax2.set_title("without transformation")

        if save_plots:
            plt.title(base_filename)
            filename = base_filename + ".png"
            full_filename = os.path.join(get_basepath(), "data", "figures", filename)
            fig.savefig(full_filename)
        plt.close()

        # plot2d(
        #         xys=[(imu_pxs, imu_pys), (gps_pxs, gps_pys)],
        #         labels=["positions from imu velocity curve", "positions from gps velocity curve"],
        #         title=base_filename,
        #         show_plots=show_plots,
        #         savefig=True
        #     )
