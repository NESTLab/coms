import unittest
import os
import pathlib
from mapmerge.constants import FREE, OCCUPIED, UNKNOWN
from mapmerge.merge_utils import augment_map, detect_fault, load_mercer_map, acceptance_index, pad_maps, resize_map
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

    def test_merge_padding(self: unittest) -> None:
        """
        Helper for handling merges across varying resolution maps
        """
        # create pair of maps with different resolutions
        map1 = INTEL_TEST_MAP
        map2 = resize_map(map1, (map1.shape[0]//2, map1.shape[1]//2))
        
        map1_pad, map2_pad = pad_maps(map1, map2)

        # map1 (larger) should not have been changed
        self.assertTrue(np.alltrue(map1_pad == map1))

        # map2 should have the same shape as map1 and same # of known/unknown cells
        self.assertTrue(map1_pad.shape == map2_pad.shape)
        num_occupied_original = np.sum(np.where(map2 == OCCUPIED, 1, 0))
        num_free_original = np.sum(np.where(map2 == FREE, 1, 0))
        num_occupied_pad = np.sum(np.where(map2_pad == OCCUPIED, 1, 0))
        num_free_pad = np.sum(np.where(map2_pad == FREE, 1, 0))
        self.assertTrue(num_free_original == num_free_pad)
        self.assertTrue(num_occupied_original == num_occupied_pad)

        # TODO @cjmclaughlin test for integration in ArGos/Gazebo env.

    def test_fail_merge(self: unittest) -> None:
        """
        Assert that failure checking catches invalid map merges
        """
        # positive example - SIFT for instance should have successful merge in most basic case
        map1, map2, map2_transform = recover_transformation(sift_mapmerge)
        self.assertTrue(detect_fault(map1, map2, map2_transform))  # returns True on successful merge

        # negative example 1 - merge results in empty image
        unknown_mask = np.ones_like(map2_transform) * UNKNOWN
        self.assertRaises(AssertionError, detect_fault, map1, map2, unknown_mask)

        # negative example 2 - merge results in information loss (walls)
        # for example, replace all walls with unknown 
        failure_map1 = np.where(map2_transform == OCCUPIED, UNKNOWN, map2_transform)
        self.assertRaises(AssertionError, detect_fault, map1, map2, failure_map1)

        # negative example 3 - merge results in information loss (overall)
        # replace 30% of free cells with unknown
        noise = np.random.choice([0, 1], size=map2_transform.shape, p=[0.3, 0.7])
        failure_map2 = np.where(noise == 1, UNKNOWN, map2_transform)
        self.assertRaises(AssertionError, detect_fault, map1, map2, failure_map2)

        # TODO @cjmclaughlin install further integration tests if initial simulation deems it necessary


if __name__ == '__main__':
    unittest.main()
