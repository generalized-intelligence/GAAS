import cv2 as cv
import numpy as np

m = np.ones((480, 752), dtype=np.uint8)
m[0:100, :] = 0
m = m * 255

cv.imwrite("mask.png", m)
