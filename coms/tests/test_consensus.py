import unittest
import os
import pathlib
from mapmerge.constants import FREE, OCCUPIED, UNKNOWN
from mapmerge.merge_utils import augment_map, combine_aligned_maps, detect_fault, load_mercer_map, acceptance_index, pad_maps, resize_map
from mapmerge.service import mapmerge_pipeline
from mapmerge.keypoint_merge import sift_mapmerge, orb_mapmerge
from mapmerge.hough_merge import hough_mapmerge
from mapmerge.ros_utils import pgm_to_numpy
import numpy as np

import matplotlib.pyplot as plt

TEST_DIR = pathlib.Path(__file__).parent.absolute()

# load 'difficult' map with curved walls and intricate details
PATH_TO_INTEL_TEST_MAP = os.path.join(TEST_DIR, 'test_data', 'intel.txt')
INTEL_TEST_MAP = load_mercer_map(PATH_TO_INTEL_TEST_MAP)


def generate_n_maps(n=4, map=INTEL_TEST_MAP):
    robot_maps = [INTEL_TEST_MAP]
    x_size, y_size = int(map.shape[0]//(n/2)), int(map.shape[1]//(n/2))
    print(x_size)
    for i in range(n-1):
        map_aug, _ = augment_map(map, shift_limit=0, rotate_limit=5)
        # mask out quadrants of the map
        robot_maps.append(map_aug)

    # mask out quadrants of the map
    robot_maps[0][:x_size, :y_size] = UNKNOWN
    robot_maps[1][:x_size, y_size::] = UNKNOWN
    robot_maps[2][x_size::, :y_size] = UNKNOWN
    robot_maps[3][x_size::, y_size::] = UNKNOWN

    fig, axes = plt.subplots(1, 5)
    axes[0].imshow(map, cmap="gray")
    axes[1].imshow(robot_maps[0], cmap="gray")
    axes[2].imshow(robot_maps[1], cmap="gray")
    axes[3].imshow(robot_maps[2], cmap="gray")
    axes[4].imshow(robot_maps[3], cmap="gray")
    plt.show()



if __name__ == "__main__":
    generate_n_maps()