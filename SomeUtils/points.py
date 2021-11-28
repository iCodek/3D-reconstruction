from datetime import datetime

from open3d import *


class PointsViewer(object):

    def __init__(self, windowName="window", width=1280, height=720):
        self.vis = Visualizer()
        self.vis.create_window('Aligned Column', width=width, height=720)
        self.pointCloud = PointCloud()
        self.vis.add_geometry(self.pointCloud)
        self.isShow = False
        self.scale = 1 / 255

    def show(self, points, color):
        start = datetime.now()
        self.pointCloud.clear()
        self.pointCloud.points = Vector3dVector(points)
        self.pointCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        if not self.isShow:
            self.isShow = True
            self.reset_view_point()
        if color is not None:
            self.pointCloud.colors = Vector3dVector(color * self.scale)
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()
        print("Process Time : {0}".format(datetime.now() - start))

    """重置视角"""

    def reset_view_point(self):
        if self.isShow:
            self.vis.reset_view_point(True)
            print("视角已重置")
