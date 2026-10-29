import math
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'slam/fast_lio'))
from run_fast_lio import PACKET, pose_payload


class FastLioBridgeTests(unittest.TestCase):
    def test_identity_pose_is_converted_from_flu_to_frd(self):
        data, initial = pose_payload([1, 2, 3], [1, 0, 0, 0], None, 123456, 7, 1)
        fields = PACKET.unpack(data)
        self.assertEqual(fields[:4], (b'FLIO', 7, 1, 123456))
        np.testing.assert_allclose(fields[4:11], [1, -2, -3, 1, 0, 0, 0], atol=1e-7)
        self.assertEqual(fields[-1], 165)
        np.testing.assert_allclose(initial, np.eye(3))

    def test_yaw_and_lever_arm_are_transformed(self):
        _, initial = pose_payload([0, 0, 0], [1, 0, 0, 0], None, 1, 1, 1)
        h = math.sqrt(.5)
        data, _ = pose_payload([1, 2, 3], [h, 0, 0, h], initial, 2, 1, 2)
        fields = PACKET.unpack(data)
        np.testing.assert_allclose(fields[4:11], [.997, -2.003, -3, h, 0, 0, -h], atol=1e-6)

    def test_invalid_pose_is_rejected(self):
        for p, q in [([float('nan'), 0, 0], [1, 0, 0, 0]),
                     ([0, 0, 0], [0, 0, 0, 0]), ([0, 0, 0], [float('inf'), 0, 0, 0])]:
            with self.subTest(p=p, q=q), self.assertRaises(ValueError):
                pose_payload(p, q, None, 1, 1, 1)


if __name__ == '__main__':
    unittest.main()
