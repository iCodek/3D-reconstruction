import cv2
import numpy as np
from cv2 import aruco

# 生成aruco标记
# 加载预定义的字典
dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)

# 生成标记
markerImage = np.zeros((400, 400), dtype=np.uint8)
for i in range(20):
    markerImage = aruco.drawMarker(dictionary, i, 150, markerImage, 1)
    cv2.imwrite("../marker" + str(i) + ".png", markerImage)
