# Bug that sometimes appears on several computers
try:
    import cv2
except ImportError:
    import cv2.cv2 as cv2

import math
import numpy as np


class Image:
    def __init__(self, img_):
        self.img = img_

    def draw_line(self, line):
        x1, y1, x2, y2 = line
        cv2.line(self.img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    def draw_rectangle(self, rect):
        x, y, w, h = rect
        cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 0, 0), 3)


class Sensor:
    def __init__(self):
        self.img = None

    def snapshot(self):
        if self.img is not None:
            return Image(self.img.copy())

    def get_frame(self, filename):
        self.img = cv2.imread(filename)
        return Image(self.img.copy())