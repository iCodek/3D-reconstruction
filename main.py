import cv2

from SomeUtils.CreatePly import *
from camera import Camera
from threshFilter import ThreshFilter

if __name__ == '__main__':
    camera = Camera(point=True)
    thresh = 50
    tf = ThreshFilter()
    i = 0
    try:
        while True:
            colorImage, depthImage, removeBgImage, points, color = camera.getOneFrame()
            dst = tf.process(colorImage)
            flat = (np.array(dst) == 0xff).flatten()
            cv2.imshow("dst", dst)
            cv2.imshow("colorImage", colorImage)
            cv2.imshow("depthImage", depthImage)
            cv2.imshow("removeBgImage", removeBgImage)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('d'):
                thresh -= 2
                tf.setThresh(thresh)
                print(thresh)

            if key & 0xFF == ord('u'):
                thresh += 2
                tf.setThresh(thresh)
                print(thresh)

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
