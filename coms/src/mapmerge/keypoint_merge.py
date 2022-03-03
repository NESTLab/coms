# TODO @cjmclaughlin further exception handling and fault detection (through sampling/ensemble method)
import numpy as np
import cv2
from mapmerge.constants import *

from mapmerge.merge_utils import blur_map, apply_warp, distance_transform, median_filter, pad_maps


def sift_mapmerge(map1, map2, transform=False):
    map1, map2 = pad_maps(map1, map2)
    try:
        sift = cv2.SIFT_create()
        merge_map1 = median_filter(blur_map(distance_transform(map1))) if transform else map1
        merge_map2 = median_filter(blur_map(distance_transform(map2))) if transform else map2
        kp1, desc1 = sift.detectAndCompute(merge_map1, None)  # TODO eval with and without blur
        kp2, desc2 = sift.detectAndCompute(merge_map2, None)
        index_params = dict(algorithm=0, trees=5)
        search_params = dict(checks=500)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)
        good_matches = []
        for m, n in matches:
            # lowes ratio
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        M, mask = cv2.estimateAffine2D(dst_pts, src_pts, confidence=0.999, ransacReprojThreshold=5.0)
        return apply_warp(map2, M)
    except:
        # failed merge
        return np.ones_like(map2, dtype=np.uint8) * UNKNOWN


import matplotlib.pyplot as plt
def orb_mapmerge(map1, map2, transform=False):
    map1, map2 = pad_maps(map1, map2)
    try:
        orb = cv2.ORB_create(nfeatures=1500, )
        merge_map1 = median_filter(blur_map(distance_transform(map1))) if transform else map1
        merge_map2 = median_filter(blur_map(distance_transform(map2))) if transform else map2
        kp1, desc1 = orb.detectAndCompute(merge_map1, None)
        kp2, desc2 = orb.detectAndCompute(merge_map2, None)
        index_params = dict(algorithm=6, table_number=12, key_size=10, multi_probe_level=2)
        search_params = dict()
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)
        good_matches = []
        for m, n in matches:
            # use slightly higher ratio for ORB
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        M, mask = cv2.estimateAffine2D(dst_pts, src_pts, confidence=0.999, ransacReprojThreshold=5.0, refineIters=30)
        return apply_warp(map2, M)
    except:
        # failed merge
        return np.ones_like(map2, dtype=np.uint8) * UNKNOWN
    
