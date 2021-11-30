from icpDemo import *
from icpDemo import *


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5

    result = o3d.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.registration.TransformationEstimationPointToPoint(False), 4, [
            o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.registration.RANSACConvergenceCriteria(4000000, 500))
    return result


if __name__ == '__main__':
    # source = o3d.io.read_point_cloud("./Data/cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("./Data/cloud_bin_1.pcd")
    threshold = 0.02
    trans_init = np.identity(4)
    source = o3d.io.read_point_cloud("./ground1.ply")
    target = o3d.io.read_point_cloud("./ground2.ply")
    draw_registration_result(source, target, trans_init)

    voxel_size = 0.01  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target,
                                                                                         voxel_size)
    result_fast = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)

    print(result_fast.transformation)
    draw_registration_result(source_down, target_down, result_fast.transformation)
    t = result_fast.transformation

    draw_registration_result(source, target, result_fast.transformation, twoColor=False)

    Asource = o3d.io.read_point_cloud("./apple1.ply")
    Atarget = o3d.io.read_point_cloud("./apple2.ply")

    draw_registration_result(Asource, Atarget, result_fast.transformation, twoColor=False)

    reg_p2l = runICP(source, target, t)
    print(reg_p2l)
    print("Transformation is:")
    print(reg_p2l.transformation)
    draw_registration_result(source, target, reg_p2l.transformation)

    draw_registration_result(Asource, Atarget, reg_p2l.transformation, twoColor=False)
