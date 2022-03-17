import unittest
import os
import pathlib
from mapmerge.constants import FREE, OCCUPIED, UNKNOWN
from mapmerge.merge_utils import augment_map, combine_aligned_maps, detect_fault, load_mercer_map, acceptance_index, pad_maps, resize_map
from mapmerge.keypoint_merge import sift_mapmerge, orb_mapmerge
from mapmerge.hough_merge import hough_mapmerge
from mapmerge.service import mapmerge_pipeline
from mapmerge.ros_utils import pgm_to_numpy
import numpy as np


TEST_DIR = pathlib.Path(__file__).parent.absolute()

# load 'difficult' map with curved walls and intricate details
PATH_TO_INTEL_TEST_MAP = os.path.join(TEST_DIR, 'test_data', 'intel.txt')
INTEL_TEST_MAP = load_mercer_map(PATH_TO_INTEL_TEST_MAP)
GMAPPING_SMALL = np.load(os.path.join(TEST_DIR, 'test_data', f'gmapping/foriegn1.npy'))

# PARAMETERS FOR TESTING - CAN LOWER NUM_TRIALS FOR FASTER CI
NUM_TRIALS = 20
TEST_ANGLES = np.linspace(start=5, stop=180, num=NUM_TRIALS)

def recover_transformation(method="hough", map=INTEL_TEST_MAP, fixed_angle=None, scale_process=False):
    """
    Apply Rotation+Translation to Random Map and Recover Original

    merge_fn: function with signature (map1: ndarray, map2: ndarray) --> map2_transform
        -- i.e. a function that finds a transformation from map2 onto map 1

    output: map1, map2, map2'
    """
    map1 = map  # original map
    map2, _ = augment_map(map, shift_limit=0.03, rotate_limit=360, fixed_angle=fixed_angle, fixed_dx=0.03, fixed_dy=0.03)  # augmented map
    map2_transform = mapmerge_pipeline(map1, map2, method=method, scale_process=scale_process)  # recovered map
    return map1, map2, map2_transform

class TestMerge(unittest.TestCase):
    def test_sift_merge(self: unittest, target_iou: float = 0.9) -> None:
        """
        SIFT merge IoU should be > 0.9 for all transformations
        """
        ious = []
        for i in range(NUM_TRIALS):
            map1, map2, map2_transform = recover_transformation(method="sift", fixed_angle=TEST_ANGLES[i])
            ious.append(acceptance_index(map1, map2_transform))
        print("SIFT MIOU", np.mean(ious))
        self.assertGreaterEqual(np.mean(ious), target_iou)
        self.assertTrue(not np.alltrue(map2_transform == UNKNOWN))

    def test_orb_merge(self: unittest, target_iou: float = 0.9) -> None:
        """
        ORB merge IoU should be > 0.9 for all transformations
        """
        ious = []
        for i in range(NUM_TRIALS):
            map1, map2, map2_transform = recover_transformation(method="orb", fixed_angle=TEST_ANGLES[i])
            ious.append(acceptance_index(map1, map2_transform))
        print("ORB MIOU", np.mean(ious))
        self.assertGreaterEqual(np.mean(ious), target_iou)
        self.assertTrue(not np.alltrue(map2_transform == UNKNOWN))

    def test_hough_merge(self: unittest, target_iou: float = 0.8) -> None:
        """
        Hough merge IoU should be > 0.8 for all transformations
        """
        ious = []
        for i in range(NUM_TRIALS):
            map1, map2, map2_transform = recover_transformation(method="hough", fixed_angle=TEST_ANGLES[i])
            ious.append(acceptance_index(map1, map2_transform))
        print("Hough MIOU", np.mean(ious))
        self.assertGreaterEqual(np.mean(ious), target_iou)

    def test_pipeline_scale_process(self: unittest) -> None:
        """
        test scale-process for merge pipeline
        """
        # first with hough
        noscale_ious = []
        for i in range(NUM_TRIALS // 2):
            map1, map2, map2_transform = recover_transformation(method="hough", fixed_angle=TEST_ANGLES[i])
            noscale_ious.append(acceptance_index(map1, map2_transform))
        scale_ious = []
        for i in range(NUM_TRIALS // 2):
            map1, map2, map2_transform = recover_transformation(method="hough", fixed_angle=TEST_ANGLES[i], scale_process=True)
            scale_ious.append(acceptance_index(map1, map2_transform))
        print(f"Hough IoU: {np.mean(noscale_ious)}, Hough+Scale IoU: {np.mean(scale_ious)} ")
        self.assertGreaterEqual(np.mean(scale_ious), np.mean(noscale_ious))
        # try KP method as well
        noscale_ious = []
        for i in range(NUM_TRIALS // 2):
            map1, map2, map2_transform = recover_transformation(method="orb", fixed_angle=TEST_ANGLES[i])
            noscale_ious.append(acceptance_index(map1, map2_transform))
        scale_ious = []
        for i in range(NUM_TRIALS // 2):
            map1, map2, map2_transform = recover_transformation(method="orb", fixed_angle=TEST_ANGLES[i], scale_process=True)
            scale_ious.append(acceptance_index(map1, map2_transform))
        print(f"KP IoU: {np.mean(noscale_ious)}, KP+Scale IoU: {np.mean(scale_ious)} ")
        self.assertGreaterEqual(np.mean(scale_ious), np.mean(noscale_ious))

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

    # def test_simulation_merge(self: unittest) -> None:
    #     """
    #     Test merge using initial data from simulation
    #     """
        # old_maps = [ os.path.join(TEST_DIR, 'test_data', f'old_map{i}.npy') for i in range(9) ]
        # maps = [ os.path.join(TEST_DIR, 'test_data', f'map_{i}.npy') for i in range(9) ]

        # for i in range(9):
        #     fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(12, 10))
        #     merge = orb_mapmerge(maps[i], old_maps[i])
        #     merge = combine_aligned_maps(merge, maps[i])
        #     axes[0].imshow(maps[i])
        #     axes[1].imshow(old_maps[i])
        #     axes[2].imshow(merge)
        #     plt.show()

if __name__ == '__main__':
    unittest.main()
