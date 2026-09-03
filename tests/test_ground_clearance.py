import unittest

import numpy as np

from general_motion_retargeting.params import GROUND_CLEARANCE_DICT
from scripts.smplx_to_robot_stream import OnlineQposPostprocessor


def _processor_with_clearance(ground_clearance):
    processor = OnlineQposPostprocessor.__new__(OnlineQposPostprocessor)
    processor.smooth_alpha = 1.0
    processor.height_adjust = True
    processor.root_origin_offset = False
    processor.ground_clearance = ground_clearance
    processor.prev_qpos = None
    processor.xy_origin = None
    processor.is_x02lite = False
    processor._forward_kinematics_for_height = lambda _q: -0.10
    return processor


class GroundClearanceTests(unittest.TestCase):
    def test_linglong2_uses_documented_ground_clearance(self):
        self.assertEqual(GROUND_CLEARANCE_DICT["linglong2"], 0.075)

    def test_ground_clearance_raises_robot_by_configured_amount(self):
        qpos = np.zeros(7, dtype=np.float32)
        baseline = _processor_with_clearance(0.0).process(qpos)
        linglong2 = _processor_with_clearance(GROUND_CLEARANCE_DICT["linglong2"]).process(qpos)

        self.assertAlmostEqual(float(linglong2[2] - baseline[2]), 0.075, places=6)

    def test_default_clearance_preserves_original_height_behavior(self):
        qpos = np.zeros(7, dtype=np.float32)
        result = _processor_with_clearance(0.0).process(qpos)

        self.assertAlmostEqual(float(result[2]), 0.10, places=6)


if __name__ == "__main__":
    unittest.main()
