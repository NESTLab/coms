import numpy as np
import matplotlib.pyplot as plt
import cv2
from mapmerge.constants import *

def load_mercer_map(txt_path, dtype=np.uint8):
    """
    read mercer-format map (.txt file) into ndarray
    """
    map = np.loadtxt(txt_path, dtype=dtype)
    if dtype != np.uint8:
        map[map == FREE] = FREE_FLOAT
        map[map == UNKNOWN] = UNKNOWN_FLOAT
    return map

def resize_map(map, dsize):
    """
    nearest neighbor resizing
    """
    return cv2.resize(map, dsize=dsize, interpolation=cv2.INTER_NEAREST)

def augment_map(map, shift_limit=0.1, rotate_limit=360, fill=UNKNOWN, scale_noise=False):
    """
    apply set of random image augmentation to map image
    return augmented map, as well as parameters for augmentation
    """
    x, y = map.shape[0], map.shape[1]
    center = y/2, x/2
    angle = np.random.uniform(low=-rotate_limit, high=rotate_limit)
    # angle = 45  # hard code for consistency
    M_rotation = cv2.getRotationMatrix2D(center=center, angle=angle, scale=1.0)
    rotated_map = apply_warp(map, M_rotation, fill=fill)
    shift_prop_x = np.random.uniform(low=-shift_limit, high=shift_limit)
    translation_x = shift_prop_x * x
    shift_prop_y = np.random.uniform(low=-shift_limit, high=shift_limit)
    translation_y = shift_prop_y * y
    M_translation = np.float32([
        [1, 0, translation_x],
        [0, 1, translation_y]
    ])
    augmented_map = apply_warp(rotated_map, M_translation, fill=fill)
    if scale_noise:
        augmented_map = cv2.resize(augmented_map, dsize=(map.shape[1]*2, map.shape[0]*2), interpolation=cv2.INTER_NEAREST)
        augmented_map = cv2.medianBlur(augmented_map, ksize=3)
        augmented_map = cv2.resize(augmented_map, dsize=(map.shape[1], map.shape[0]), interpolation=cv2.INTER_NEAREST)
    augment_dict = {"translation_x":translation_x, "translation_y":translation_y, "angle":angle}
    return augmented_map, augment_dict


def get_training_sample(map_filename, shift_limit=0.1, rotate_limit=360):
    """
    from a filename,
    generate a sample of the loaded map image and an augmented copy
    (e.g. translation + rotation as in Carpin et al.)
    """
    map = load_mercer_map(map_filename)
    aug_map, labels = augment_map(map, shift_limit=shift_limit, rotate_limit=rotate_limit)
    return map, aug_map

def show_samples():
    """
    Display set of samples from current training set (defined in constants)
    """
    for map_idx in range(len(TRAIN_FILENAMES)):
        intel = load_mercer_map(TRAIN_FILENAMES[map_idx])
        plt.imshow(intel, cmap="gray")
        plt.axis("off")
        plt.show()

def apply_warp(map, M, fill=UNKNOWN):
    """
    Apply affine transformation matrix to map
    """
    map_warp = np.copy(map)
    x, y = map.shape[0], map.shape[1]
    map_warp = cv2.warpAffine(src=map_warp, M=M, dsize=(y, x), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=fill)
    return map_warp

def combine_aligned_maps(map1, map2):
    """
    combine explored/unexplored regions where possible
    """
    combined = np.copy(map1)
    combined = np.where(combined == UNKNOWN, map2, map1)
    return combined

def agreement(map1, map2):
    """
    "Agreement" from Carpin et al.
    # of pairs which are both occupied/occupied or free/free
    """
    agree = np.count_nonzero((map1 == OCCUPIED) & (map2 == OCCUPIED))
    agree += np.count_nonzero((map1 == FREE) & (map2 == FREE))
    return agree    

def disagreement(map1, map2):
    """
    "Disagreement" from Carpin et al.
    # of pairs which are occupied/free in each map, and vice-versa
    """
    disagree = np.count_nonzero((map1 == OCCUPIED) & (map2 == FREE))
    disagree += np.count_nonzero((map1 == FREE) & (map2 == OCCUPIED))
    return disagree

def acceptance_index(map1, map2):
    """
    Acceptance index from Carpin et al., aka IoU
    """
    a = agreement(map1, map2)
    d = disagreement(map1, map2)
    iou = 0 if a == 0 else (a / (a+d))
    return iou