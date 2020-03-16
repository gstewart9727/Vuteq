# Filename      : register.py
# Version       : 0.0.0
# Version Date  :
# Programmer    : Gabriel Stewart
# Description   : This file contains the source code for the Open3D registration process. The functions
#                 for loading point clouds, downsampling, global registraion, icp registration, and 
#                 displaying results are contained here.

# Import libraries
import open3d as o3d
import numpy as np
import copy
import time
from scipy.spatial import distance
import scipy
import scipy.io
import ctypes   

# Global variable determining level of output of application
verbose = False


# Function      : draw_registration_Result
# Parameters    : source/target - point cloud datasets
#                 transformation - matrix containing calculated registration transformation
# Returns       : None
# Description   : This function creates a visualization window for the passed source clouds
def draw_registration_result(source, target, transformation):

    # Create copy of point clouds to be displayed to preserve originals
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)

    # (Optional) Paint clouds referenced color
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])

    # Apply transformation to temporary source point cloud and display
    source_temp.transform(transformation)
    geometries = ([source_temp, target_temp])
    visuals(geometries)


# Function      : visuals
# Parameters    : clouds - List containing point clouds to be displayed
# Returns       : None
# Description   : This function creates a visualization window for the passed source clouds
def visuals(clouds):

    # Create an instance of the visualizer window
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1410, left=0, top=0)

    # Iterate through list of points clouds and add them to visualizer
    for geometry in clouds:
        vis.add_geometry(geometry)

    # Display visualizer window and destroy it when user closes it
    vis.run()
    vis.destroy_window()


# Function      : preprocess_point_cloud
# Parameters    : pcd - Point cloud to be processed
#                 voxel-size - unit size for downsampling ratio
# Returns       : None
# Description   : This function uniformly reduces the number of points in a cloud to
#                 increase the effeciency of further registration operations
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd_down, pcd_fpfh


# Function      : prepare_dataset
# Parameters    : pcd - Point cloud to be processed
#                 voxel-size - unit size for downsampling ratio
# Returns       : Processed point clouds
# Description   : This function uniformly reduces the number of points in a cloud to
#                 increase the efficiency of further registration operations
def prepare_dataset(voxel_size):
    print(":: Load two point clouds")
    target = o3d.io.read_point_cloud("..\Training Mold\Main-Refined.ply")
    # source = o3d.io.read_point_cloud("..\Training Mold\CamData.ply")
    # target = o3d.io.read_point_cloud("..\Training Mold\Main - Cloud.ply")
    source = o3d.io.read_point_cloud("..\Training Mold\Training_mold.ply")
    source.scale(1000)
    target.scale(1)

    # Prepare bounding box for cropping
    bounding_points = o3d.io.read_point_cloud("..\Training Mold\cropped_1.ply")
    bbox = o3d.geometry.OrientedBoundingBox(o3d.geometry.OrientedBoundingBox.create_from_points(bounding_points.points))
    
    # IF set to verbose mode, display unregistered point clouds
    if (verbose == True):
        print(np.asarray(o3d.geometry.OrientedBoundingBox.get_box_points(bbox)))
        geometries = ([bbox, source])
        visuals(geometries)

    # Perform cropping
    source_crop = source.crop(bbox)

    # Perform processing on point clouds before registration process continues
    source_down, source_fpfh = preprocess_point_cloud(source_crop, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


# Function      : execute_global_registration
# Parameters    : source_down/target_down - downsampled pointclouds
# Returns       : Result of transformation
# Description   : This function performs a large global registration on the two point clouds using RANSAC algorithm.
#                 This step must be performed prior to ICP registration, to get both point 
#                 clouds close enough together for ICP to work properly and quickly
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):

    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


# Function      : refine_registration
# Parameters    : source_down/target_down - downsampled pointclouds
#                 result_ransac - The transformation result of the global registration
# Returns       : Result of transformation
# Description   : This function performs an ICP registration to the results of the global RANSAC registration.
#                 The result will be two point clouds that are aligned closely enough to perform deviation analysis.
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result

def display(occurrences):
    warning_data = "{} Deviation points detected. Check mold surface.".format(occurrences)
    ctypes.windll.user32.MessageBoxW(0, warning_data, "Warning", 1)

def Mbox(title, text, style):
    return ctypes.windll.user32.MessageBoxW(0, text, title, style)


# Function      : run
# Parameters    : devThreshVal - The deviation threshold value. Used to set distance for two points to be considered deviated.
#                 devTolVal - The deviation tolerance value. Used to determine how many deviated points are permitted
#                   before the alarm is sounded
#                 verbosity - A boolean value used to determine the level of output from the registration process.
# Returns       : None
# Description   : This function calls the functions and performs calculations required for the full registration process.
def run(self, devThreshVal, devTolVal, verbosity):

    # Set verbosity level
    global verbose
    verbose = verbosity

    # Start timer
    start = time.time()

    # Prepare datasets
    voxel_size = 4
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size)

    # Perform global registration
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    # Display global registration results
    if (verbose == True):
        print(result_ransac)
        draw_registration_result(source_down, target_down, result_ransac.transformation)

    # Perform icp registration
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                    voxel_size, result_ransac)
    # Display icp registration results
    if (verbose == True):
        print(result_icp)
        draw_registration_result(source_down, target_down, result_icp.transformation)

    # Record time for registration process
    time_finish = time.time() - start
    print("Total time is %.3f sec.\n" % time_finish)
    
    # Perform transformation on camera point cloud data
    source_down.transform(result_icp.transformation)

    # Calculated ueclidian distance between point pairs for use in deviation analysis
    distances = distance.cdist(np.asarray(source_down.points), np.asarray(target_down.points), 'euclidean')
    distances = np.min(np.array(distances), axis=1)
    print ('Source Data Points \n')
    print(np.asarray(source_down.points))
    print ('Target Data Points \n')
    print(np.asarray(target_down.points))
    print ('Distances \n')
    print(distances)

    # Count number of points deviating beyond threshold
    source_down.paint_uniform_color([1, 0.706, 0])
    target_down.paint_uniform_color([0, 0.651, 0.929])
    a = np.logical_and(distances > float(devThreshVal), distances < 50)
    occurrences = np.count_nonzero(a == True)

    # Display number of points beyond deviation threshold and sound warning if required
    if occurrences > float(devTolVal): # Allow for some false positives
        display(occurrences)

    # Show highlighted deviation points
    color_array = np.asarray(source_down.colors)
    color_array[a, :] = [1, 0, 0]
    source_down.colors = o3d.utility.Vector3dVector(color_array)
    if (verbose == True):
        geometries = ([source_down, target_down])
        visuals(geometries)