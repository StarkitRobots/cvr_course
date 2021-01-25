# Bug that sometimes appears on several computers
try:
    import cv2
except ImportError:
    import cv2.cv2 as cv2

import math
import numpy as np


class Blob:
    def __init__(self, x_, y_, w_, h_):
        self.x_ = x_
        self.y_ = y_
        self.w_ = w_
        self.h_ = h_

    def x(self):
        return self.x_

    def y(self):
        return self.y_

    def w(self):
        return self.w_

    def h(self):
        return self.h_

    def cx(self):
        return int(self.x_ + self.w_ / 2)

    def cy(self):
        return int(self.y_ + self.h_ / 2)

    def rect(self):
        return self.x_, self.y_, self.w_, self.h_


class Line:
    def __init__(self, x1_, y1_, x2_, y2_, theta_):
        self.x1_ = x1_
        self.y1_ = y1_
        self.x2_ = x2_
        self.y2_ = y2_
        self.theta_ = theta_

    def x1(self):
        return self.x1_

    def y1(self):
        return self.y1_

    def x2(self):
        return self.x2_

    def y2(self):
        return self.y2_

    def theta(self):
        return self.theta_

    def line(self):
        return self.x1_, self.y1_, self.x2_, self.y2_


class Image:
    def __init__(self, img_):
        self.img = img_

    def find_blobs(self, th, pixels_threshold, area_threshold, merge, margin):
        low_th = (int(th[0] * 2.55), th[2] + 128, th[4] + 128)
        high_th = (int(th[1] * 2.55), th[3] + 128, th[5] + 128)

        labimg = cv2.cvtColor(self.img, cv2.COLOR_BGR2LAB)

        mask = cv2.inRange(labimg, low_th, high_th)

        #cv2.imshow ("a", mask)

        output = cv2.connectedComponentsWithStats(mask, 8, cv2.CV_32S)

        #labels_count = output      [0]
        #labels       = output      [1]
        stats        = output      [2]
        #labels_count, labels, stats = output[:3]
        sz = stats.shape[0]

        blobs = []

        for label_num in range(1, sz):
            area = stats[label_num, cv2.CC_STAT_AREA]
            
            if area >= pixels_threshold:
                new_blob = Blob(stats[label_num, cv2.CC_STAT_LEFT],
                                stats[label_num, cv2.CC_STAT_TOP],
                                stats[label_num, cv2.CC_STAT_WIDTH],
                                stats[label_num, cv2.CC_STAT_HEIGHT])

                #print ("append", area)
                blobs.append(new_blob)

        return blobs

    def binary(self, th):
        low  = (th[0], th[2], th[4])
        high = (th[1], th[3], th[5])

        mask = cv2.inRange(self.img, low, high)

        sh = mask.shape

        result = np.zeros((sh[0], sh[1], 3), self.img.dtype)

        for i in range(0, 3):
            result[:, :, i] = mask.copy()

        return result

    def find_lines(self):
        # - Почему Колумб приплыл в Америку, а не в Индию?
        # - Потому что он плавал по одометрии

        gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)

        #cv2.imshow ("a", edges)

        #lines = cv2.HoughLines(edges, 5, np.pi / 18, 20)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 150)

        resultant_lines = []

        #print (lines)

        for line in lines:
            x1, y1, x2, y2 = line [0]

            theta = math.atan((y2 - y1) / (x2 - x1))

            new_line = Line(x1, y1, x2, y2, theta)

            resultant_lines.append(new_line)

        return resultant_lines

    def draw_line(self, line):
        x1, y1, x2, y2 = line
        cv2.line(self.img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    def draw_rectangle(self, rect):
        x, y, w, h = rect
        cv2.rectangle(self.img, (x, y), (x+w, y+h), (255, 0, 0), 3)

class Sensor:
    def __init__(self, filename_):
        self.filename = filename_
        self.img = cv2.imread(self.filename)

    def snapshot(self):
        return Image(self.img.copy())