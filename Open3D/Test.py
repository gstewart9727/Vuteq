import open3d as o3d
import numpy as np
import copy
import time
from scipy.spatial import distance
import scipy
import scipy.io
import ctypes   

#scipy.spatial.distance.directed_hausdorff


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706, 0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


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


def prepare_dataset(voxel_size):
    print(":: Load two point clouds")
    source = o3d.io.read_point_cloud("..\Training Mold\CamData.ply")
    target = o3d.io.read_point_cloud("..\Training Mold\Main - Cloud.ply")
    source.scale(1000)
    target.scale(1)

    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


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


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    return result

def Mbox(title, text, style):
    return ctypes.windll.user32.MessageBoxW(0, text, title, style)


if __name__ == "__main__":

    # Start timer
    start = time.time()

    voxel_size = 4 
    source, target, source_down, target_down, source_fpfh, target_fpfh = \
            prepare_dataset(voxel_size)

    # Perform global registration
    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxel_size)

    # Display global registration results
    print(result_ransac)
    # draw_registration_result(source_down, target_down, result_ransac.transformation)

    # Perform icp registration
    result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh,
                                     voxel_size)
    # Display icp registration results
    print(result_icp)
    # draw_registration_result(source_down, target_down, result_icp.transformation)

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

    # Highlight points that deviate beyond a set threshold and display them
    source_down.paint_uniform_color([1, 0.706, 0])
    target_down.paint_uniform_color([0, 0.651, 0.929])
    a = np.logical_and(distances > 3.5, distances < 50)
    color_array = np.asarray(source_down.colors)
    color_array[a, :] = [1, 0, 0]
    source_down.colors = o3d.utility.Vector3dVector(color_array)
    o3d.visualization.draw_geometries([source_down, target_down])

    # Display number of points beyond deviation threshold and sound warning if required
    occurrences = np.count_nonzero(a == True)
    if occurrences > 10: # Allow for some false positives
        warning_data = "{} Deviation points detected. Check mold surface.".format(occurrences)
        ctypes.windll.user32.MessageBoxW(0, warning_data, "Warning", 1)
    print (type(a))
