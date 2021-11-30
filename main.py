import cv2
from open3d import *
from SomeUtils.createPly import *
from SomeUtils.points import PointsViewer
from arucoFilter import ArucoFilter
from camera import Camera
from icpFunc import *


if __name__ == '__main__':
    camera = Camera(point=True)
    thresh = 50
    tf = ArucoFilter()
    i = 0
    pv = PointsViewer()
    result = PointCloud()

    resultPv = PointsViewer(windowName="result")

    filterPointCloudList = []

    try:
        while True:
            _, colorImage, depthImage, removeBgImage, points, color, m = camera.getOneFrame()

            dst, centers, ids, _= tf.process(colorImage)
            cv2.imshow("dst", dst)
            flat = (dst == 0xff).flatten()[m]

            cv2.imshow("dst", dst)
            cv2.imshow("colorImage", colorImage)
            #cv2.imshow("depthImage", depthImage)
            #cv2.imshow("removeBgImage", removeBgImage)

            pointCould = pv.show(points, color)

            key = cv2.waitKey(500)

            if key & 0xFF == ord('a') and centers:

                filterPoint = PointCloud()
                filterPoint.points = Vector3dVector(points[flat])
                filterPoint.colors = Vector3dVector(color[flat])
                if filterPointCloudList:
                    target = filterPointCloudList[-1]
                    registration = execute_global_registration(filterPoint, target, 1)
                    print(registration.fitness)
                    print(registration.inlier_rmse)
                    print(registration.transformation)
                    #draw_registration_result(filterPoint, target, registration.transformation)

                    # registration = execute_plane_registration(filterPoint, target, 0.1, trans_init)
                    # print(registration.fitness)
                    # print(registration.inlier_rmse)
                    # print(registration.transformation)

                    pointCould.transform(registration.transformation)

                    draw_registration_result(filterPoint.transform(registration.transformation), target)

                filterPointCloudList.append(filterPoint)
                print(pointCould)
                resultPv.showPointCloud(pointCould)

            if key & 0xFF == ord('r'):
                pv.reset_view_point()
                print("reset view point")

            if key & 0xFF == ord('s'):
                # 异步保存ply点云文件，用CloudCompare软件打开
                i += 1
                asyncSavePly(points[flat], color[flat], "ground" + str(i) + ".ply")
                asyncSavePly(points, color, "apple" + str(i) + ".ply")

            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

    finally:
        camera.stopCamera()
