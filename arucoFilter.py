import cv2
import numpy as np
from cv2 import aruco

from Interface.filter import ImageFilter


class ArucoFilter(ImageFilter):

    def process(self, image, *args, **kwargs):

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 设置预定义的字典
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        # 使用默认值初始化检测器参数
        parameters = aruco.DetectorParameters_create()
        # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        mask = np.zeros(image.shape[:2])
        if ids is not None:
            for corner in corners:
                cv2.fillPoly(mask, np.int32(corner), color=255)
        # 画出标志位置
        # aruco.drawDetectedDiamonds(mask, corners, ids, borderColor=255)
        return mask
