import unittest
import os
import pathlib
from mapmerge.merge_utils import augment_map, load_mercer_map, acceptance_index
from mapmerge.keypoint_merge import sift_mapmerge, orb_mapmerge
from mapmerge.hough_merge import hough_mapmerge
import numpy as np

TEST_DIR = pathlib.Path(__file__).parent.absolute()
PATH_TO_INTEL_TEST_MAP = os.path.join(TEST_DIR, 'test_data', 'intel.txt')
# load 'difficult' map with curved walls and intricate details
INTEL_TEST_MAP = load_mercer_map(PATH_TO_INTEL_TEST_MAP)


def recover_transformation(merge_fn):
    """
    Apply Rotation+Translation to Random Map and Recover Original

    merge_fn: function with signature (map1: ndarray, map2: ndarray) --> map2_transform
        -- i.e. a function that finds a transformation from map2 onto map 1

    output: map1, map2, map2'
    """
    map1 = INTEL_TEST_MAP  # original map
    map2, _ = augment_map(INTEL_TEST_MAP, shift_limit=0.05, rotate_limit=360)  # augmented map
    map2_transform = merge_fn(map1, map2)  # recovered map
    return map1, map2, map2_transform


class TestMerge(unittest.TestCase):
    def test_sift_merge(self: unittest, num_trials: int = 25, target_iou: float = 0.95) -> None:
        """
        SIFT merge IoU should be > 0.95 for all transformations
        """
        ious = []
        for _ in range(num_trials):
            map1, map2, map2_transform = recover_transformation(sift_mapmerge)
            ious.append(acceptance_index(map1, map2_transform))
        self.assertGreaterEqual(np.mean(ious), target_iou)

    def test_orb_merge(self: unittest, num_trials: int = 25, target_iou: float = 0.95) -> None:
        """
        ORB merge IoU should be > 0.95 for all transformations
        """
        ious = []
        for _ in range(num_trials):
            map1, map2, map2_transform = recover_transformation(orb_mapmerge)
            ious.append(acceptance_index(map1, map2_transform))
        self.assertGreaterEqual(np.mean(ious), target_iou)

    def test_hough_merge(self: unittest, num_trials: int = 25, target_iou: float = 0.8) -> None:
        """
        Hough merge IoU should be > 0.8 for all transformations
        """
        ious = []
        for _ in range(num_trials):
            map1, map2, map2_transform = recover_transformation(lambda m1, m2: hough_mapmerge(m1, m2, num=5, robust=True))
            ious.append(acceptance_index(map1, map2_transform))
        self.assertGreaterEqual(np.mean(ious), target_iou)


if __name__ == '__main__':
    unittest.main()
