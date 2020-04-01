# Filename      : interactive_visualization.py
# Version       : 0.1
# Version Date  : 2020-03-29
# Programmer    : Gabriel Stewart
# Description   : This file contains the source code for the Open3D cropping functionality. A live camera
#                 stream is opened and then used to gather a snapshot of pointcloud data which is then used for 
#                 gathering of the ROI.
# Sources
# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details
# examples/Python/Advanced/interactive_visualization.py

import numpy as np
import copy
import open3d as o3d
import pyrealsense2 as rs
import cv2
import keyboard


# Function      : crop_geometry
# Parameters    : None
# Returns       : None
# Description   : This function gathers a snapshot of live camera data and crops it according to user selection
def crop_geometry(done_queue):

    # Display instructions in console
    print("Manual geometry cropping")
    print("1) Press 'Y' twice to align geometry with negative direction of y-axis")
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    voxel_size = 4

    # Start streaming
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    # Create visualizer window with specified size and dimensions
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1395, height=670, left=10, top=100)
    pointcloud = o3d.geometry.PointCloud()

    # Variable that indicates if geometry has already been added to visualizer
    geom_added = False
        
    # Run loop until instructed to close
    while True:

        # Wait for frames to arrive in pipeline
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)
        profile = frames.get_profile()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        # If frames did not arrive within period, reset
        if not depth_frame or not color_frame:
            continue
        
        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        img_depth = o3d.geometry.Image(depth_image)
        img_color = o3d.geometry.Image(color_image)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
        
        # Create pointcloud using camera frames aligned as rgbd image
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pointcloud.points = pcd.points
        pointcloud.scale(1000)

        # If geometry has not yet been added to visualizer, do so
        if geom_added == False:
            done_queue.put('stage|Live Data Stream')
            vis.add_geometry(pointcloud)
            geom_added = True

        # Update visualizer and poll for interaction events every cycle
        vis.update_geometry(pointcloud)
        vis.poll_events()
        vis.update_renderer()
        cv2.imshow('RGB View', color_image)

        # Exit loop if 'q' key is pressed
        if keyboard.is_pressed('q'):  
            break
        
    # When active loop has completed, stop the camera pipeline and destroy visualization windows
    pipeline.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
    del vis

    # Preprocess point cloud and send it to Open3D function for cropping
    done_queue.put('stage|ROI Cropping')
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(width=1395, height=670, left=10, top=100)    
    vis.add_geometry(pointcloud)
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
