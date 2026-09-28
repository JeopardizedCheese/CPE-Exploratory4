import copy
import unittest

import cv2
import numpy as np

from reference_guard import ReferenceGuard
from vision import Detector
from tests.test_find_zones import field, hsv_cfg
from tests.test_vision import configuration, color


class ReferenceTests(unittest.TestCase):
    def config(self):
        cfg = configuration()
        cfg['hsv'] = hsv_cfg()['hsv']
        cfg['arena'] = {'size_mm': [2300, 1690], 'mm_per_px': 2,
                        'corners_px': [[0, 0], [1149, 0], [1149, 844], [0, 844]]}
        cfg['vision']['reference_guard'] = {'enabled': True}
        return cfg

    def test_stones_do_not_look_like_camera_motion(self):
        guard = ReferenceGuard(self.config(), field(False))
        self.assertEqual(guard.check(field(True))['status'], 'ok')

    def test_shifted_field_is_detected(self):
        bg = field(False)
        moved = cv2.warpAffine(bg, np.float32([[1, 0, 8], [0, 1, 6]]), (bg.shape[1], bg.shape[0]))
        result = ReferenceGuard(self.config(), bg).check(moved)
        self.assertEqual(result['status'], 'moved')
        self.assertAlmostEqual(result['median_shift_mm'], 20, delta=3)

    def test_missing_markers_are_unverified(self):
        bg = field(False)
        self.assertEqual(ReferenceGuard(self.config(), bg).check(np.full_like(bg, 180))['status'], 'unverified')

    def test_verdict_must_last_hold_s_before_it_counts(self):
        cfg = self.config()
        cfg['vision']['reference_guard']['hold_s'] = 2.0
        bg = field(False)
        moved = cv2.warpAffine(bg, np.float32([[1, 0, 8], [0, 1, 6]]), (bg.shape[1], bg.shape[0]))
        guard = ReferenceGuard(cfg, bg)
        first = guard.check(moved, now=10.0)
        self.assertEqual((first['status'], first['pending']), ('ok', 'moved'))
        self.assertEqual(guard.check(moved, now=11.9)['status'], 'ok')
        self.assertEqual(guard.check(moved, now=12.1)['status'], 'moved')
        # one good frame resets the timer
        self.assertEqual(guard.check(bg, now=12.2)['status'], 'ok')
        self.assertEqual(guard.check(moved, now=13.0)['status'], 'ok')

    def test_field_profile_ignores_small_jitter(self):
        # field values: 25 mm limit; at 2 mm/px a 10 px (20 mm) shift is jitter, 15 px (30 mm) is not
        cfg = self.config()
        cfg['vision']['reference_guard'].update(max_shift_mm=25, min_markers=2)
        bg = field(False)
        guard = ReferenceGuard(cfg, bg)
        shift = lambda px: cv2.warpAffine(bg, np.float32([[1, 0, px], [0, 1, 0]]), (bg.shape[1], bg.shape[0]))
        self.assertEqual(guard.check(shift(10))['status'], 'ok')
        self.assertEqual(guard.check(shift(15))['status'], 'moved')

    def test_reference_change_revokes_sticky_targets(self):
        cfg = self.config()
        cfg['vision']['sticky_frames'] = 5
        bg = field(False)
        frame = bg.copy()
        cv2.rectangle(frame, (560, 400), (579, 419), color(20), -1)
        d = Detector(cfg, bg)
        for _ in range(3):
            result = d.process(frame)
        self.assertTrue(any(o.stable and o.isolated for o in result[1]))
        moved = cv2.warpAffine(frame, np.float32([[1, 0, 8], [0, 1, 6]]), (frame.shape[1], frame.shape[0]))
        result = d.process(moved)
        self.assertEqual(result[3], 'reference_moved')
        self.assertFalse(any(o.stable for o in result[1]))
        self.assertEqual(d.previous_targets, [])


if __name__ == '__main__':
    unittest.main()
