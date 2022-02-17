import numpy as np
import unittest
import pathlib
import os
from compression.rle import to_rle, decode_rle
from compression.quadtree import to_quadtree, decode_quadtree
from mapmerge.merge_utils import load_mercer_map

TEST_DIR = pathlib.Path(__file__).parent.absolute()
PATH_TO_INTEL_TEST_MAP = os.path.join(TEST_DIR, 'test_data', 'intel.txt')
INTEL_TEST_MAP = load_mercer_map(PATH_TO_INTEL_TEST_MAP)


class TestCompression(unittest.TestCase):
    def test_rle_compression(self: unittest) -> None:
        rle_encoded = to_rle(INTEL_TEST_MAP)
        rle_decoded = decode_rle(rle_encoded, map_shape=INTEL_TEST_MAP.shape)
        self.assertTrue(np.alltrue(rle_decoded == INTEL_TEST_MAP))

    def test_quadtree_compression(self: unittest) -> None:
        quadtree_encoded = to_quadtree(INTEL_TEST_MAP)
        quadtree_decoded = decode_quadtree(quadtree_encoded, map_shape=INTEL_TEST_MAP.shape)
        self.assertTrue(np.alltrue(quadtree_decoded == INTEL_TEST_MAP))


if __name__ == '__main__':
    unittest.main()
