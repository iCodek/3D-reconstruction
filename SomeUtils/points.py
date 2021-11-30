from datetime import datetime
import cv2
from open3d import *


class PointsViewer(object):

    def __init__(self, windowName="window", width=1280, height=720):
        self.vis = Visualizer()
        self.vis.create_window(windowName, width=width, height=720)
        self.pointCloud = PointCloud()
        self.vis.add_geometry(self.pointCloud)
        self.isShow = False
        self.scale = 1 / 255

    def show(self, points, color):
        start = datetime.now()
        self.pointCloud.clear()
        self.pointCloud.points = Vector3dVector(points)
        # self.pointCloud.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        if not self.isShow:
            self.isShow = True
            self.reset_view_point()
        if color is not None:
            self.pointCloud.colors = Vector3dVector(color * self.scale)
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()
        # print("Process Time : {0}".format(datetime.now() - start))
        return self.pointCloud

    def showPointCloud(self, pointCloud, voxel_size=0.005):
        if pointCloud is not None:
            temp = voxel_down_sample(self.pointCloud + pointCloud, voxel_size)
            self.pointCloud.clear()
            self.pointCloud += temp
            if not self.isShow:
                self.isShow = True
                self.reset_view_point()
        self.vis.update_geometry()
        self.vis.poll_events()
        self.vis.update_renderer()
        return self.pointCloud

    """重置视角"""

    def reset_view_point(self):
        if self.isShow:
            self.vis.reset_view_point(True)
            print("视角已重置")

class Point(object):
    def __init__(self, x=0, y=0, z=0, id=0):
        self.x = x
        self.y = y
        self.z = z
        self.id = id

    def toString(self):
        return "{0} x:{1:.1f},y:{2:.1f},z:{3:.1f}".format(self.id, self.x, self.y, self.z)

def getTransMatrix(src, dst):
    srcSumX = srcSumY = srcSumZ = 0
    dstSumX = dstSumY = dstSumZ = 0
    pointsNum = len(src)
    for i in range(pointsNum):
        srcSumX += src[i].x
        srcSumY += src[i].y
        srcSumZ += src[i].z

        dstSumX += dst[i].x
        dstSumY += dst[i].y
        dstSumZ += dst[i].z

    centerSrc = Point()
    centerSrc.x = srcSumX / pointsNum
    centerSrc.y = srcSumY / pointsNum
    centerSrc.z = srcSumZ / pointsNum

    centerDst = Point()
    centerDst.x = dstSumX / pointsNum
    centerDst.y = dstSumY / pointsNum
    centerDst.z = dstSumZ / pointsNum

    srcMat = np.zeros((3, pointsNum))
    dstMat = np.zeros((3, pointsNum))

    for i in range(pointsNum):

        srcMat[0, i] = src[i].x - centerSrc.x
        srcMat[1, i] = src[i].y - centerSrc.y
        srcMat[2, i] = src[i].z - centerSrc.z

        dstMat[0, i] = dst[i].x - centerDst.x
        dstMat[1, i] = dst[i].y - centerDst.y
        dstMat[2, i] = dst[i].z - centerDst.z

    matS = srcMat.dot(dstMat.T)

    w, u, v = cv2.SVDecomp(matS)
    matTemp = u.dot(v)
    det = cv2.determinant(matTemp)


    matM = np.array([[1,0,0],
                     [0,1,0],
                     [0,0,det]])


    matR = v.T.dot(matM).dot(u.T)

    datR = matR.reshape(-1,)

    delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2])

    delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);

    delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);


    T = np.array([
    [matR[0, 0], matR[0, 1], matR[0, 2], delta_X],
    [matR[1, 0], matR[1, 1], matR[1, 2], delta_Y],
    [matR[2, 0], matR[2, 1], matR[2, 2], delta_Z],
    [0, 0, 0, 1]])

    return T

if __name__ == '__main__':
    p1 = Point(100,0,0)
    p2 = Point(0,100,0)
    p3 = Point(0,0,100)

    src = []
    src.append(p1)
    src.append(p2)
    src.append(p3)

    p1 = Point(50,100,0)
    p2 = Point(-50,0,0)
    p3 = Point(50,0,100)

    dst = []
    dst.append(p1)
    dst.append(p2)
    dst.append(p3)

    result = getTransMatrix(src, dst)
    print(result)

