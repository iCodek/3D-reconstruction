import copy

import numpy as np
import open3d as o3d

trans_init = np.identity(4)


def runICP(source, target, current_transformation=None):
    threshold = 0.02
    print("Apply point-to-plane ICP")
    if current_transformation is None:
        current_transformation = trans_init
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    for scale in range(3):
        iter = max_iter[scale]
        radius = voxel_radius[scale]
        print([iter, radius, scale])

        print("3-1. Downsample with a voxel size %.2f" % radius)
        source_down = o3d.voxel_down_sample(source, radius)
        target_down = o3d.voxel_down_sample(target, radius)

        print("3-2. Estimate normal.")
        o3d.estimate_normals(source_down,
                             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        o3d.estimate_normals(target_down,
                             o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        print("3-3. Applying colored point cloud registration")
        result_icp = o3d.registration.registration_colored_icp(
            source_down, target_down, radius, current_transformation)
        current_transformation = result_icp.transformation
        print(result_icp)
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation)
    return result_icp


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


def draw_registration_result(source, target, transformation):
    if transformation is None:
        transformation = trans_init
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == '__main__':
    source = o3d.io.read_point_cloud("./Data/cloud_bin_0.pcd")
    target = o3d.io.read_point_cloud("./Data/cloud_bin_1.pcd")
    threshold = 0.02
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                             [-0.139, 0.967, -0.215, 0.7],
                             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    draw_registration_result(source, target, trans_init)

    print("Initial alignment")
    evaluation = o3d.registration.evaluate_registration(
        source, target, threshold, trans_init)
    print(evaluation)

    print("Apply point-to-point ICP")
    reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration=200))
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    draw_registration_result(source, target, reg_p2p.transformation)

    print("Apply point-to-plane ICP")
    reg_p2l = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPlane())
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(source, target, reg_p2l.transformation)
