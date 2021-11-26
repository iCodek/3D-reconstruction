import cv2

from Interface.filter import ImageFilter


class ThreshFilter(ImageFilter):

    def process(self, image, *args, **kwargs):
        dst = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        _, dst = cv2.threshold(dst, self.thresh, 255, cv2.THRESH_BINARY_INV)
        return dst

    def setThresh(self, thresh):
        self.thresh = thresh

    def __init__(self, thresh=50, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setThresh(thresh)
