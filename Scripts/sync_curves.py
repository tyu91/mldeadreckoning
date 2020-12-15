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

    # transformed_imu_values = np.dot(normalized_transform, np.hstack([imu_values, np.ones((num_elems, 1))]).T).T
    # # TODO: temp solution, haven't implemented RANSAC
    # centered_gps_values = gps_values

    # return transformed_imu_values[:, 0], transformed_imu_values[:, 1], \
    #     centered_gps_values[:, 0], centered_gps_values[:, 1]
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
    threshold = 1e2

    transform_inv = np.linalg.inv(transform)

    transform1 = np.dot(transform, np.hstack([imu_values, np.ones((num_elems, 1))]).T).T[:, :2]
    forward_l2_error = np.power(transform1 - gps_values, 2)

    transform2 = np.dot(transform_inv, np.hstack([gps_values, np.ones((num_elems, 1))]).T).T[:, :2]
    backward_l2_error = np.power(transform2 - imu_values, 2)

    total_l2_error = forward_l2_error + backward_l2_error

    l2_error = np.sqrt(np.power(total_l2_error[:, 0], 2) + np.power(total_l2_error[:, 1], 2))

    within_threshold = np.count_nonzero(l2_error < threshold)

    return within_threshold

def ransac(imu_values, gps_values):
    '''implementation of RANSAC algorithm to pick best fitting transformation matrix

    :rtype: best-performing transformation
    '''
    num_elems = imu_values.shape[0]
    num_trials = num_elems
    num_samples = 4
    inliers = 0
    best_transform = np.zeros((3, 3))

    for i in range(num_trials):
        indices = random.sample(range(num_elems), num_samples)
        sampled_imu_values = imu_values[indices, :]
        sampled_gps_values = gps_values[indices, :]

        # compute transform based on sample points
        transform = compute_homography(sampled_imu_values, sampled_gps_values)

        # count number of inliers using this transform
        curr_inliers_count = num_inliers(transform, imu_values, gps_values)
        if curr_inliers_count > inliers:
            inliers = curr_inliers_count
            best_transform = transform
            # print("INLIERS COUNT: ", inliers)

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
    single_file = False # perform odometry on single file vs. all files
    tag_files = False # write tags to file
    show_plots = False # plot positions
    save_plots = True # save plots to data
    is_50hz = False 
    sanity_check = False # set to true to make sure RANSAC algo works properly

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
        if "stationary" in imu_relative_path or "random" in imu_relative_path or "boosting" in imu_relative_path:
          continue
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
        if (len(imu_pxs) > 25000):
            continue
        interp_factor = 20
        imu_pxs, imu_pys, imu_pzs = interpolate_xyz(imu_pxs, imu_pys, imu_pzs, len(gps_pxs) * interp_factor)
        gps_pxs, gps_pys, gps_pzs = interpolate_xyz(gps_pxs, gps_pys, gps_pzs, len(gps_pxs) * interp_factor)
        if sanity_check:
            fake_transform = np.array([[0.5, -0.5, 3], [3, 1, 7], [0, 0, 1]])
            fake_imu =  np.random.rand(100, 2)  
            fake_gps = np.dot(fake_transform, np.hstack([fake_imu, np.ones((100, 1))]).T).T[:, :2]                                                                                                                                                
            imu_pxs, imu_pys, gps_pxs, gps_pys = compute_affine_transformation(fake_imu[:, 0], fake_imu[:, 1], None, [fake_gps[:, 0]], [fake_gps[:, 1]], None)
        else:
            imu_pxs, imu_pys, gps_pxs, gps_pys = compute_affine_transformation(imu_pxs, imu_pys, imu_pzs, gps_pxs, gps_pys, gps_pzs)

        assert(len(imu_pxs) == len(gps_pxs))

        imu_t = np.arange(0, len(imu_pxs))
        gps_t = np.arange(0, len(gps_pxs))

        plot2d(
                xys=[(imu_pxs, imu_pys), (gps_pxs, gps_pys)],
                labels=["positions from imu velocity curve", "positions from gps velocity curve"],
                title=base_filename,
                show_plots=show_plots,
                savefig=True
            )
