import numpy as np
import cv2

from mapmerge.constants import *

def numpy_2_ros(array):
    """
    returns a numpy array from a ros int8 array
    """
    array[FREE] = 100
    array[UNKNOWN] = -1
    return array

def ros_2_numpy(array):
    """
    returns a int8 array from numpy
    """
    array[100] = FREE
    array[-1] = UNKNOWN
    return array

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

    img[img >= 250] = FREE # temp
    img[(img > 0) & (img < 250)] = UNKNOWN
    return img

def numpy_to_pgm(arr):
    """
    Save as pgm, to publish to ROS node

    TODO
    """
    pass
