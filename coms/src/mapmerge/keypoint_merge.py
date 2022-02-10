import numpy as np
import cv2
from mapmerge.constants import *

from mapmerge.merge_utils import apply_warp

def blur_map(map):
    src = np.copy(map)
    blur = cv2.GaussianBlur(src,(3,3), sigmaX=1, sigmaY=1)
    return blur

def sift_mapmerge(map1, map2):
    map1, map2 = blur_map(map1), blur_map(map2)
    sift = cv2.SIFT_create()
    kp1, desc1 = sift.detectAndCompute(map1, None)
    kp2, desc2 = sift.detectAndCompute(map2, None)
    index_params = dict(algorithm = 0, trees = 5)
    search_params = dict(checks=50)
    try:
        flann = cv2.FlannBasedMatcher(index_params,search_params)
        matches = flann.knnMatch(desc1,desc2,k=2)
        good_matches = []
        for m, n in matches:
            # lowes ratio
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)
    except:
        # failed merge
        return np.ones_like(map2) * UNKNOWN
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
    return apply_warp(map2, M[:2])

def orb_mapmerge(map1, map2):
    # map1, map2 = blur_map(map1), blur_map(map2)
    orb = cv2.ORB_create(nfeatures=1000)
    print(map1.shape)
    print(map2.shape)
    kp1, desc1 = orb.detectAndCompute(map1, None)
    kp2, desc2 = orb.detectAndCompute(map2, None)
    index_params = dict(algorithm = 6, trees = 6, key_size=12, multi_probe_level = 1)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params,search_params)
    matches = flann.knnMatch(desc1,desc2,k=2)
    good_matches = []
    try:
        for m, n in matches:
            # use slightly higher ratio for ORB
            if m.distance < 0.8 * n.distance:
                good_matches.append(m)
    except:
        # failed merge
        return np.ones_like(map2) * UNKNOWN
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good_matches ]).reshape(-1,1,2)
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good_matches ]).reshape(-1,1,2)

    M, mask = cv2.findHomography(dst_pts, src_pts, cv2.RANSAC, 5.0)
    return apply_warp(map2, M[:2])