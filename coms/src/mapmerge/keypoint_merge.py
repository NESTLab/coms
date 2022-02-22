# TODO @cjmclaughlin further exception handling and fault detection (through sampling/ensemble method)
import numpy as np
import cv2
from mapmerge.constants import *

from mapmerge.merge_utils import blur_map, apply_warp, pad_maps


def sift_mapmerge(map1, map2):
    # map1, map2 = blur_map(map1), blur_map(map2)
    try:
        map1, map2 = pad_maps(map1, map2)
        sift = cv2.SIFT_create()
        kp1, desc1 = sift.detectAndCompute(blur_map(map1), None)
        kp2, desc2 = sift.detectAndCompute(blur_map(map2), None)
        index_params = dict(algorithm=0, trees=5)
        search_params = dict(checks=150)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)
        good_matches = []
        for m, n in matches:
            # lowes ratio
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        return apply_warp(map2, M[:2])
    except:
        # failed merge
        return np.ones_like(map2, dtype=np.uint8) * UNKNOWN



def orb_mapmerge(map1, map2):
    # map1, map2 = blur_map(map1), blur_map(map2)
    try:
        map1, map2 = pad_maps(map1, map2)    
        orb = cv2.ORB_create(nfeatures=1000)
        kp1, desc1 = orb.detectAndCompute(blur_map(map1), None)
        kp2, desc2 = orb.detectAndCompute(blur_map(map2), None)
        index_params = dict(algorithm=6, trees=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=150)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(desc1, desc2, k=2)
        good_matches = []
        for m, n in matches:
            # use slightly higher ratio for ORB
            if m.distance < 0.8 * n.distance:
                good_matches.append(m)
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
        return apply_warp(map2, M[:2])
    except:
        # failed merge
        return np.ones_like(map2, dtype=np.uint8) * UNKNOWN
    
