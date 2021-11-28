import open3d as o3d

from globalicp import *

if __name__ == '__main__':
    # source = o3d.io.read_point_cloud("./Data/cloud_bin_0.pcd")
    # target = o3d.io.read_point_cloud("./Data/cloud_bin_1.pcd")

    source = o3d.io.read_point_cloud("./ground1.ply")
    target = o3d.io.read_point_cloud("./ground2.ply")

    voxel_size = 0.001  # means 5cm for this dataset
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target,
                                                                                         voxel_size)

    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    print(result_fast)
    print(result_fast.transformation)
    draw_registration_result(source_down, target_down, result_fast.transformation)
    draw_registration_result(source, target, result_fast.transformation, twoColor=False)

    source = o3d.io.read_point_cloud("./apple1.ply")
    target = o3d.io.read_point_cloud("./apple2.ply")
    draw_registration_result(source, target, result_fast.transformation, twoColor=False)
