import copy

import numpy as np
import open3d as o3d


def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target])


if __name__ == '__main__':
    print("1. Load two point clouds and show initial pose")
    source = o3d.io.read_point_cloud("./Data/frag_115.ply")
    target = o3d.io.read_point_cloud("./Data/frag_116.ply")

    # draw initial alignment
    current_transformation = np.identity(4)
    draw_registration_result_original_color(source, target, current_transformation)

    # point to plane ICP
    current_transformation = np.identity(4)
    print("2. Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. Distance threshold 0.02.")
    result_icp = o3d.registration.registration_icp(
        source, target, 0.02, current_transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    print(result_icp)
    draw_registration_result_original_color(source, target,
                                            result_icp.transformation)

    # colored pointcloud registration
    # This is implementation of following paper
    # J. Park, Q.-Y. Zhou, V. Koltun,
    # Colored Point Cloud Registration Revisited, ICCV 2017
    voxel_radius = [0.04, 0.02, 0.01]
    max_iter = [50, 30, 14]
    current_transformation = np.identity(4)
    print("3. Colored point cloud registration")
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
