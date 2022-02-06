import numpy as np
import cv2

from constants import *

def pgm_to_numpy(filename):
    """
    PGM --> numpy array with value encoding scheme from our current map merging code

    Example mapping:
    pgm    numpy
    254    255 (FREE)
    204    127 (UNKNOWN)
    0      0   (OCCUPIED)
    """
    img = cv2.imread(filename, -1)
    if img is None:
        print("INVALID FILENAME", filename)
    img = np.asarray(img)
    img[img > 250] = FREE # temp
    img[(img > 0) & (img < 250)] = UNKNOWN
    return img

def numpy_to_pgm(arr):
    """
    Save as pgm, to publish to ROS node

    TODO
    """
    pass
