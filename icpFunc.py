import numpy as np
import open3d as o3d
import copy
trans_init = np.identity(4)


def draw_registration_result(source, target, transformation=None, twoColor=True):
    if transformation is None:
        transformation = trans_init
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    if twoColor:
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def execute_point_registration(source, target, threshold, trans_init):
    print("Apply point-to-point ICP")
    reg = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=200))
    return reg

def execute_plane_registration(source, target, threshold, trans_init):
    print("Apply point-to-plane ICP")
    reg = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())
    return reg


def execute_global_registration(source, target, voxel_size=0.01):

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target,
                                                                                         voxel_size)
    print("Apply Global ICP")
    distance_threshold = voxel_size * 1.5

    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


def execute_color_registration(source, target, current_transformation=None):
    result_icp = None
    print("Apply Color ICP")
    if current_transformation is None:
        current_transformation = trans_init
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    for scale in range(3):
        radius = voxel_radius[scale]

        source_down = o3d.voxel_down_sample(source, radius)
        target_down = o3d.voxel_down_sample(target, radius)

        o3d.estimate_normals(source_down,
                             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        o3d.estimate_normals(target_down,
                             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        result_icp = o3d.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation)
        current_transformation = result_icp.transformation
    return result_icp


def prepare_dataset(source, target, voxel_size):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = o3d.voxel_down_sample(pcd, voxel_size)
    radius_normal = voxel_size * 2
    o3d.estimate_normals(pcd_down,
                         o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh
