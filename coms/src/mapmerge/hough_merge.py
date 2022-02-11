import numpy as np
from skimage.transform import hough_line, hough_line_peaks
import matplotlib.pyplot as plt
import scipy
import cv2
from mapmerge.merge_utils import apply_warp, acceptance_index

from mapmerge.constants import *

def hough_spectrum_calculation(image):
    h, theta, d = hough_map_transform(image)
    spectrum = np.sum(np.square(h), axis=0)
    max = np.max(spectrum)
    spectrum = spectrum / max
    return spectrum

def render_hough_spectrum(spectrum, image):
    """
    Plot spectra and image, useful for debugging
    """
    # rendering the image is optional
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax = axes.ravel()
    ax[0].imshow(image, cmap="gray")
    ax[0].set_title('Input image')
    ax[0].set_axis_off()
    
    ax[1].set_title('Hough Spectrum')
    ax[1].set_xlabel('angle (theta)')
    ax[1].set_ylabel('hough spectrum')
    ax[1].plot(spectrum)
    plt.show()

def hough_map_transform(map):
    tested_angles = np.linspace(0, 2*np.pi, 360, endpoint=False)
    # change map to format s.t. 1 where occupied, 0 otherwise
    edge_map = np.copy(map)
    edge_map[edge_map == OCCUPIED] = 1  # temp
    edge_map[edge_map == UNKNOWN] = 0
    edge_map[edge_map == FREE] = 0
    # plt.imshow(edge_map, cmap="gray")
    # plt.show()
    h, theta, d = hough_line(edge_map, theta=tested_angles)
    return h, theta, d

def FFT_circular_cross_correlation(HTM1, HTM2):
    # circular cross correlation done with FFT for O(n log N) time complexity
    cc = np.fft.ifft(np.fft.fft(np.flip(HTM1)) * np.fft.fft(np.flip(HTM2)))
    return cc / np.max(cc)

def cross_correlation(m1_spectrum, m2_spectrum):
    cc = np.fft.ifft(np.fft.fft(m2_spectrum) * np.conjugate(np.fft.fft(m1_spectrum)))
    return cc/ np.linalg.norm(cc)

def extract_local_maximums(signal, num):
    return np.argpartition(signal, -num)[-num:]

def axis_spectrum(axis, map):
    edge_map = np.copy(map)
    # edge_map[edge_map == OCCUPIED] = 1  # temp
    edge_map[edge_map == UNKNOWN] = FREE
    edge_map = 255 - edge_map
    # edge_map[edge_map == FREE] = 0
    # edge_map[edge_map == 1] = 255
    spect = np.sum(edge_map, axis=axis)
    return spect / np.max(spect)

def hough_mapmerge(map1, map2, num=4, robust=False, eps=3):
    """
    produces best possible accuracy merges given two maps

    Inputs:
        num: number of hypothesis to test (default: 4 as in paper)
        robust: whether or not to generate 2 extra hypotheses for each primary candidate
        eps: value used to generate additional hypothesis in robust mode, (candidate +/- eps)
             quick experiments show that eps between 2 and 5 may be good
    """
    x, y = map1.shape[0], map1.shape[1]
    center = y/2, x/2
    best_map = None
    best_acpt = -1
    best_params = None

    HS_M1 = hough_spectrum_calculation(map1)
    HS_M2 = hough_spectrum_calculation(map2)

    CC_M1_M2 = FFT_circular_cross_correlation(HS_M1, HS_M2)
    local_max = extract_local_maximums(CC_M1_M2, num)

    # add r+/-eps into candidates for robust version
    robust_max = []
    if robust:
        for r in local_max:
            robust_max.append(r)
            robust_max.append(r+eps)
            robust_max.append(r-eps)
        local_max = robust_max
    
    SX_M1 = axis_spectrum(0, map1)
    SY_M1 = axis_spectrum(1, map1)

    map3 = None
    for rot in local_max:
        M_rotation = cv2.getRotationMatrix2D(center=center, angle=-rot, scale=1.0)
        map3 = apply_warp(map2, M_rotation, fill=UNKNOWN)  # dont cheat lol
        SX_M3 = axis_spectrum(0, map3)
        SY_M3 = axis_spectrum(1, map3)

        CCX = scipy.signal.correlate(SX_M1, SX_M3, method="fft")
        CCY = scipy.signal.correlate(SY_M1, SY_M3, method="fft")
        
        best_dx = (CCX.shape[0] + (1 - SX_M1.shape[0]) - np.argmax(CCX))
        best_dy = (CCY.shape[0] + (1 - SY_M1.shape[0]) - np.argmax(CCY))
        
        M_translation = np.float32([
            [1, 0, -best_dx],
            [0, 1, -best_dy]
        ])
        cand_map = apply_warp(map3, M_translation, fill=UNKNOWN)
        acpt = acceptance_index(map1, cand_map)
        
        if acpt > best_acpt:
            best_acpt = acpt
            best_map = cand_map
            
    return best_map