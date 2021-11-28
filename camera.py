import math

import cv2
import numpy as np
import pyrealsense2 as rs


class Camera:
    """需要滤除的距离"""

    def __init__(self, clipping_distance_in_meters=10, point=False):
        self.pipeline = rs.pipeline()
        self.pc = rs.pointcloud()
        self.point = point
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        self.clipping_distance = clipping_distance_in_meters / depth_scale
        self.align = rs.align(rs.stream.color)

    """"获取一副图片，深度图片，移除背景的图片"""

    def getOneFrame(self):

        for i in range(10):
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            grey_color = 153
            depth_image_3d = np.dstack(
                (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color,
                                  color_image)

            # Render images
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            depth_colormap = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)

            if self.point:

                points = self.pc.calculate(aligned_depth_frame)
                self.pc.map_to(color_frame)

                v, t = points.get_vertices(), points.get_texture_coordinates()
                verts = np.asarray(v).view(np.float32).reshape(-1, 3)  # xyz
                verts = np.asarray(verts, dtype=np.float64)
                texcoords = np.asarray(t).view(np.float32).reshape(-1, 2)  # uv
                h, w = color_image.shape[:2]
                v, u = (texcoords * (w, h) + 0.5).astype(np.uint32).T
                im = (v >= 0) & (v < w)
                jm = (u >= 0) & (u < h)
                m = im & jm
                color = color_image[u, v]

                return color_image, depth_colormap, bg_removed, verts[m], color[m], m
            else:
                return color_image, depth_colormap, bg_removed, None, None
        raise Exception("相机获取图片失败")

    """关闭相机"""

    def stopCamera(self):
        self.pipeline.stop()


class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)
