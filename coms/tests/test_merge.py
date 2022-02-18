import unittest

from msg import Merge
from mapmerge.merge_utils import load_mercer_map
import numpy as np

INTEL_TEST_MAP = load_mercer_map("coms/tests/test_data/intel.txt")

# 1. show produce/consume payload retain data integrity
# 2. handle ???
class TestMerge(unittest.TestCase):
    def test_produce_payload(self: unittest.TestCase) -> None:
        pass

    def test_consume_payload(self: unittest.TestCase) -> None:
        pass

    def test_payload_integrity(self: unittest.TestCase) -> None:
        m = Merge(map=INTEL_TEST_MAP, source=("192.168.0.1", 1234), destination=("192.168.5.3", 8081))
        m_bytes = m.produce_payload()
        m_decoded = Merge(map=np.empty((1,1))).consume_payload(m_bytes)
        self.assertTrue(np.alltrue(m_decoded.map == m.map))
        self.assertEqual(m.source, m_decoded.source)
        self.assertEqual(m.target, m_decoded.target)

    def test_handle(self: unittest.TestCase) -> None:
        pass

    def test_create_occupancy_msg(self: unittest.TestCase) -> None:
        pass


if __name__ == '__main__':
    unittest.main()
