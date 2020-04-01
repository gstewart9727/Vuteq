# Filename      : register.py
# Version       : 0.1
# Version Date  : 2020-03-26
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
import win32gui
import win32con
import os
import Surface_Analysis as sa
import keyboard
import pyrealsense2 as rs
import sys


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
    vis.create_window(width=1395, height=670, left=10, top=100)

    # Iterate through list of points clouds and add them to visualizer
    for geometry in clouds:
        vis.add_geometry(geometry)

    # Display visualizer window and destroy it when user closes it
    vis.run()
    vis.destroy_window()

def windowEnumerationHandler(hwnd, top_windows):
    top_windows.append((hwnd, win32gui.GetWindowText(hwnd)))


# Function      : preprocess_point_cloud
# Parameters    : pcd - Point cloud to be processed
#                 voxel-size - unit size for downsampling ratio
# Returns       : None
# Description   : This function uniformly reduces the number of points in a cloud to
#                 increase the effeciency of further registration operations
def preprocess_point_cloud(pcd, voxel_size):
    # print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    # print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    # print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
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
def prepare_source(voxel_size, cropFile, pipeline, align):

    # Start camera and pipeline
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)
    profile = frames.get_profile()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    img_depth = o3d.geometry.Image(depth_image)
    img_color = o3d.geometry.Image(color_image)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
    
    # Gather pointcloud from camera stream
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)

    # Modify the cloud for visibility in visualizer
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    pcd.paint_uniform_color([1, 0.706, 0])
    pcd.scale(1000)

    # Prepare bounding box for cropping
    cropROI = o3d.io.read_point_cloud(cropFile, format='ply')
    bbox = cropROI.get_oriented_bounding_box()

    # If set to verbose mode, display unregistered point clouds
    if (verbose):
        geometries = ([bbox, pcd])
        visuals(geometries)

    # Perform cropping
    source_crop = pcd.crop(bbox)

    # Perform processing on point clouds before registration process continues
    source_down, source_fpfh = preprocess_point_cloud(source_crop, voxel_size)
    return source_crop, source_down, source_fpfh


# Function      : execute_global_registration
# Parameters    : source_down/target_down - downsampled pointclouds
# Returns       : Result of transformation
# Description   : This function performs a large global registration on the two point clouds using RANSAC algorithm.
#                 This step must be performed prior to ICP registration, to get both point 
#                 clouds close enough together for ICP to work properly and quickly
def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):

    distance_threshold = voxel_size * 1.5
    # print(":: RANSAC registration on downsampled point clouds.")
    # print("   Since the downsampling voxel size is %.3f," % voxel_size)
    # print("   we use a liberal distance threshold %.3f." % distance_threshold)
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
    # print(":: Point-to-plane ICP registration is applied on original point")
    # print("   clouds to refine the alignment. This time we use a strict")
    # print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result


# Function      : run
# Parameters    : devThreshVal - The deviation threshold value. Used to set distance for two points to be considered deviated.
#                 devTolVal - The deviation tolerance value. Used to determine how many deviated points are permitted
#                   before the alarm is sounded
#                 verbosity - A boolean value used to determine the level of output from the registration process.
# Returns       : None
# Description   : This function calls the functions and performs calculations required for the full registration process.
def run(task_queue, done_queue, targetFile, cropFile):

    print('STARTED')

    # Prepare target pointcloud from stored ply file
    try:
        voxel_size = 4
        target = o3d.io.read_point_cloud(targetFile)
        target.scale(1)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    except RuntimeError as e:
        done_queue.put('error|Target file not found. Check target filename.|1')
        sys.exit([e])

    # Configure depth and color streams
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        pipeline.start(config)
        align = rs.align(rs.stream.color)
    except RuntimeError as e:
        done_queue.put('error|Device not connected. Check camera connection.|2')
        sys.exit([e])

    # Start loop to wait for task requests from parent
    while True:

        # Receive message from parent over queue
        message = task_queue.get()
        fields = message.split('|')

        # Exit loop if quit has been specified
        if (fields[0] == 'quit'):
            break

        # Gather variables from message
        devThreshVal = fields[0]
        devTolVal = fields[1]
        if (fields[2] == 'verbose'):
            verbose = True
        elif (fields[2] == 'quick'):
            verbose = False
        
        # Start timer
        start = time.time()
        total = start

        # Prepare source data from camera
        try:
            source, source_down, source_fpfh = prepare_source(voxel_size, cropFile, pipeline, align)
        except RuntimeError as e:
            done_queue.put('error|Error preparing camera data. Validate crop file and camera connection.|3')
            sys.exit([e])

        # Send information about pointclouds
        if (verbose):
            # print ('Source Data Points \n')
            done_queue.put('sourcePoints|Points: {}'.format(len(source.points)))
            done_queue.put('sourcePointsDS|DS Points: {}'.format(len(source_down.points)))
            # print ('Target Data Points \n')
            done_queue.put('targetPoints|Points: {}'.format(len(target.points)))
            done_queue.put('targetPointsDS|DS Points: {}'.format(len(target_down.points)))

        # Paint both clouds uniform colors to easily differentiate
        source_down.paint_uniform_color([1, 0.706, 0])
        target_down.paint_uniform_color([0, 0.651, 0.929])

        # Perform global registration
        result_ransac = execute_global_registration(source_down, target_down,
                                                    source_fpfh, target_fpfh,
                                                    voxel_size)

        # Output time taken for global transformation
        done_queue.put('time|RANSAC time:\t\t{:.4f} seconds'.format(time.time() - start))
        start = time.time()
            
        # Display global registration results
        if (verbose):
            done_queue.put('stage|Global Registration')
            draw_registration_result(source_down, target_down, result_ransac.transformation)

        # Perform icp registration
        result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                        voxel_size, result_ransac)

        # Output time taken for ICP registration
        done_queue.put('time|ICP time:\t\t{:.4f} seconds'.format(time.time() - start))
        start = time.time()

        # Display icp registration results
        if (verbose):
            done_queue.put('stage|ICP Registration')
            draw_registration_result(source_down, target_down, result_icp.transformation)
        
        # Perform transformation on camera point cloud data
        source_down.transform(result_icp.transformation)

        # Calculated ueclidian distance between point pairs for use in deviation analysis
        distances = distance.cdist(np.asarray(source_down.points), np.asarray(target_down.points), 'euclidean')
        distances = np.min(np.array(distances), axis=1)

        # Count number of points deviating beyond threshold
        source_down.paint_uniform_color([1, 0.706, 0])
        target_down.paint_uniform_color([0, 0.651, 0.929])
        a = np.logical_and(distances > float(devThreshVal), distances < 50)
        occurrences = np.count_nonzero(a == True)

        # Output time taken to calculate deviation results
        done_queue.put('time|Deviation time:\t{:.4f} seconds'.format(time.time() - start))
        done_queue.put('time|Total time:\t\t{:.4f} seconds'.format(time.time() - total))
        start = time.time()

        # Display number of points beyond deviation threshold and sound warning if required
        if occurrences > float(devTolVal): # Allow for some false positives
            done_queue.put('result|Deviated Points: |{}'.format(occurrences))

        # Show highlighted deviation points
        color_array = np.asarray(source_down.colors)
        color_array[a, :] = [1, 0, 0]
        source_down.colors = o3d.utility.Vector3dVector(color_array)
        if (verbose):
            done_queue.put('stage|Deviation Display')
            geometries = ([source_down, target_down])
            visuals(geometries)

        # Notify parent that application is finished
        done_queue.put('stage|Idle')