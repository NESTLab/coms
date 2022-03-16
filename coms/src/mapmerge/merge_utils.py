import numpy as np
import matplotlib.pyplot as plt
import cv2
from mapmerge.constants import *
from scipy import ndimage

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

def pad_maps(map1, map2):
    """
    Pad maps with UNKOWN such that they are the same size
    """
    # get greatest size in each dimension
    x, y = max([map1.shape[0], map2.shape[0]]), max([map1.shape[1], map2.shape[1]]) 
    
    # create array of UNKNOWNS with size x, y
    pad_map1 = np.ones(shape=(x, y), dtype=np.uint8) * UNKNOWN
    pad_map2 = np.ones(shape=(x, y), dtype=np.uint8) * UNKNOWN

    # fill original data from maps, starting at origin (0,0)
    pad_map1[:map1.shape[0], :map1.shape[1]] = map1
    pad_map2[:map2.shape[0], :map2.shape[1]] = map2

    return pad_map1, pad_map2

def binarize_image(map):
    """
    0 where wall
    1 elsewhere
    """
    binary_map = np.copy(map)
    binary_map[binary_map == OCCUPIED] = 1
    binary_map[binary_map != 1] = 0
    binary_map = np.where(binary_map == 1, 0, 1)
    return binary_map

def distance_transform(map):
    """
    euclidean distance transform of map
    """
    dist = ndimage.distance_transform_edt(binarize_image(map))
    dist /= dist.max()
    dist = np.sqrt(dist)
    dist = dist * 255
    dist = dist.astype(np.uint8)
    return dist

def blur_map(map, ksize=3):
    """
    apply gaussian blur to map. can be useful for keypoint merge methods to remove noise.
    """
    src = np.copy(map)
    blur = cv2.GaussianBlur(src, ksize=(ksize, ksize), sigmaX=1, sigmaY=1)
    return blur

def median_filter(map, ksize=3):
    """
    median filter image
    """
    src = np.copy(map)
    median = cv2.medianBlur(src, ksize=3)
    return median

def detect_fault(map1, map2, merged):
    """
    given a pair of maps and their predicted merge
    evaluate if the merged map is valid

    optional args: 
        - quick_check (bool): whether or not abbreviated fault check should be used
    """
    # check 1: merged map should retain at least 90% of information (non-unknown cells)
    # relative to the initial map with the most information
    def count_information(map):
        num_occ = np.sum(np.where(map == OCCUPIED, 1, 0))
        num_free = np.sum(np.where(map == FREE, 1, 0))
        num_info = num_occ + num_free
        return num_occ, num_free, num_info
    
    map1_occ, map1_free, map1_info = count_information(map1)
    map2_occ, map2_free, map2_info = count_information(map2)
    merge_occ, merge_free, merge_info = count_information(merged)

    info_to_retain = 0.9 * max(map1_info, map2_info)
    assert merge_info >= info_to_retain, "Attempted merge lost too much information (Overall)."

    # additionally, we can check that individual types of information were not corrupted
    occ_to_retain = 0.75 * max(map1_occ, map2_occ)
    free_to_retain = 0.75 * max(map1_free, map2_free)
    assert merge_occ >= occ_to_retain, "Attempted merge lost too much information (Occupied)."
    assert merge_free >= free_to_retain, "Attempted merge lost too much information (Free)."

    return True  # passed all checks

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

def apply_warp(map, M, fill=UNKNOWN):
    """
    Apply affine transformation matrix to map
    """
    map_warp = np.copy(map)
    x, y = map.shape[0], map.shape[1]
    map_warp = cv2.warpAffine(src=map_warp, M=M, dsize=(y, x), flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=fill)
    return map_warp

def combine_aligned_maps(merged_map, original_map):
    """
    combine explored/unexplored regions where possible

    this should be called with pairs of maps that have already been merged (aligned) with one another
    """
    combined = np.copy(merged_map)
    combined = np.where(combined == UNKNOWN, original_map, merged_map)
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