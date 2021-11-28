from datetime import datetime

import pyrealsense2 as rs
from open3d import *

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

# Start streaming
profile = pipeline.start(config)

# Streaming loop
try:
    geometrie_added = False
    vis = Visualizer()
    vis.create_window('Aligned Column', width=1280, height=720)
    pointcloud = PointCloud()
    while True:
        pointcloud.clear()

        dt0 = datetime.now()
        # Wait for the next set of frames from the camera
        frames_1 = pipeline.wait_for_frames()
        # Align depth and color frame
        depth_1 = frames_1.get_depth_frame()
        color_1 = frames_1.get_color_frame()
        if not depth_1 or not color_1:  # or not depth_2 or not color_2:
            continue
        # Create RGBD
        color_raw = Image(np.array(color_1.get_data()))
        depth_raw = Image(np.array(depth_1.get_data()))
        rgbd_image = create_rgbd_image_from_color_and_depth(color_raw, depth_raw, convert_rgb_to_intensity=False)
        # Get intrinsic
        tt_1 = profile.get_stream(rs.stream.depth)
        intr_1 = tt_1.as_video_stream_profile().get_intrinsics()
        pinhole_camera_intrinsic = PinholeCameraIntrinsic(intr_1.width, intr_1.height, intr_1.fx,
                                                          intr_1.fy, intr_1.ppx, intr_1.ppy)
        # Create Point cloud from rgbd
        pcd_1 = create_point_cloud_from_rgbd_image(rgbd_image, pinhole_camera_intrinsic)
        pcd_1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        # draw_geometries([pcd_1])
        pointcloud += pcd_1
        if not geometrie_added:
            vis.add_geometry(pointcloud)
            geometrie_added = True
        vis.update_geometry()
        vis.poll_events()
        # vis.update_renderer()
        process_time = datetime.now() - dt0
        fps = 1.0 / process_time.total_seconds()
        print("Fps : {0}".format(fps))
        print(process_time)
finally:
    pipeline.stop()
