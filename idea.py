import cv2
from open3d import *
from SomeUtils.createPly import *
from SomeUtils.points import PointsViewer, Point, getTransMatrix
from arucoFilter import ArucoFilter
from camera import Camera
from icpFunc import *
import pyrealsense2 as rs
from SomeUtils.createPly import *


if __name__ == '__main__':
    camera = Camera(point=True)

    tf = ArucoFilter()

    #pv = PointsViewer()
    result = PointCloud()
    scale = 1 / 255
    resultPv = PointsViewer(windowName="result")

    firstPoints = None
    firstPointCloud = None
    pointCloud = PointCloud()
    times = 0
    try:
        pp2 = PointCloud()
        while True:
            aligned_frames, colorImage, depthImage, removeBgImage, points, color, m = camera.getOneFrame()

            dst, centers, ids = tf.process(colorImage)
            flat = (dst == 0xff).flatten()[m]
            pointMap = {}
            #pointCould = pv.show(points[flat], color[flat])
            #pointCould = pv.show(points, color)

            pointCloud.clear()
            pointCloud.points = Vector3dVector(points)
            pointCloud.colors = Vector3dVector(color[:, [2, 1, 0]] * scale)
            resultPv.showPointCloud(None, 0.005)
            for i in range(len(centers)):
                y = centers[i][0][0]
                x = centers[i][0][1]
                z = aligned_frames.get_distance(y, x)

                if z:
                    point = rs.rs2_deproject_pixel_to_point(camera.color_intrinsics, [y, x], z)
                    pt = Point(point[0], point[1], point[2], ids[i][0])
                    colorImage = cv2.putText(colorImage, pt.toString(), (y, x), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
                    pointMap[pt.id] = pt
            if pointMap:

                if firstPoints is not None:

                    idSet = firstPoints.keys() & pointMap.keys()

                    if len(idSet) >= 5:
                        t, L = getTransMatrix([pointMap[k] for k in idSet], [firstPoints[k] for k in idSet])
                        print(L)
                        if L < 3e-5:


                            #print(t)
                            # pp1 = PointCloud()
                            # pts = np.array([[pointMap[k].x, pointMap[k].y, pointMap[k].z] for k in idSet])
                            # pp1.points = Vector3dVector(pts)
                            #
                            # pp1.paint_uniform_color([0, 0, 1])
                            #print(pp2)
                            #pp1.transform(t)

                            #draw_registration_result(pp1, pp2, None, twoColor=True)
                            #draw_registration_result(pp1, firstPointCloud, None, twoColor=True)
                            #o3d.io.write_point_cloud("firstPointCloud.pcd", firstPointCloud)

                            #o3d.io.write_point_cloud("pp2.pcd", PointCloud(pp2))
                            #print("saving")
                            #draw_registration_result(pp1, pp2 + firstPointCloud, t, twoColor=False)
                            #draw_registration_result(pointCould, firstPointCloud, t, twoColor=False)
                            pointCloud.transform(t)
                            resultPv.showPointCloud(pointCloud, 0.003)
                else:
                    if len(pointMap) >= 15:
                        times += 1
                    if len(pointMap) >= 15 and times > 5:
                        firstPoints = pointMap
                        firstPointCloud = PointCloud(pointCloud)
                        # pts2 = np.array([[firstPoints[k].x, firstPoints[k].y, firstPoints[k].z] for k in firstPoints])
                        # pp2.points = Vector3dVector(pts2)
                        # pp2.paint_uniform_color([1, 0, 0])
                        #draw_registration_result(pp2, firstPointCloud, None, twoColor=True)

                        #draw_registration_result(pp2, pp2, None, twoColor=False)
                        print("第一张采集完成")



            cv2.imshow("dst", dst)


            cv2.imshow("colorImage", colorImage)
            #cv2.imshow("depthImage", depthImage)
            # cv2.imshow("removeBgImage", removeBgImage)

            key = cv2.waitKey(100)

            if key & 0xFF == ord('r'):
                pv.reset_view_point()
                print("reset view point")

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        camera.stopCamera()

