import unittest
import cv2
import numpy as np
from target_lock import TargetLock
from vision import Detector
from tests.test_pile import pile_config
from tests.test_vision import color


def t(color, x, y, conf=.6, approach=None):
    d = {'color': color, 'x': x, 'y': y, 'confidence': conf}
    if approach is not None:
        d['approach_deg'] = approach
    return d


class TargetLockTests(unittest.TestCase):
    def test_flicker_between_two_stones_keeps_first(self):
        lock = TargetLock()
        violet, cyan = t(1, 100, 100, .5), t(2, 140, 100, .6)
        self.assertEqual(lock.update([violet], 0.0)['color'], 1)
        for i, frame in enumerate([[cyan], [violet], [cyan], [], [cyan]]):   # vision flickers
            self.assertEqual(lock.update(frame, 0.1 * (i + 1))['color'], 1)

    def test_lost_after_timeout_then_relocks(self):
        lock = TargetLock(max_missing_s=1.0)
        lock.update([t(1, 100, 100)], 0.0)
        self.assertIsNotNone(lock.update([], 0.9))
        self.assertIsNone(lock.update([], 1.2))
        self.assertEqual(lock.reason, 'lost')
        self.assertEqual(lock.update([t(2, 300, 300)], 1.3)['color'], 2)

    def test_occlusion_does_not_time_out(self):
        lock = TargetLock(max_missing_s=1.0)
        lock.update([t(1, 100, 100)], 0.0)
        self.assertIsNotNone(lock.update([], 5.0, occluded=True))
        self.assertEqual(lock.reason, 'occluded')

    def test_colour_change_releases(self):
        lock = TargetLock()
        lock.update([t(1, 100, 100)], 0.0)
        lock.update([], 0.1, observations=[{'color': 3, 'x': 102, 'y': 99}])
        self.assertEqual(lock.reason, 'colour changed')

    def test_follows_small_moves_and_keeps_colour(self):
        lock = TargetLock()
        lock.update([t(1, 100, 100, approach=0)], 0.0)
        got = lock.update([t(1, 110, 104, approach=10)], 0.1)
        self.assertEqual((got['x'], got['approach_deg'], got['color']), (110, 10, 1))

    def test_done_frees_lock(self):
        lock = TargetLock()
        lock.update([t(1, 100, 100)], 0.0)
        lock.done()
        self.assertIsNone(lock.target)


class StickyVisionTests(unittest.TestCase):
    def test_target_survives_short_dropout(self):
        cfg = pile_config()
        cfg['vision']['sticky_frames'] = 3
        bg = np.full((240, 320, 3), 140, np.uint8)
        frame = bg.copy()
        cv2.rectangle(frame, (130, 100), (149, 119), color(20), -1)
        whitened = frame.copy()
        cv2.rectangle(whitened, (130, 100), (149, 119), (255, 255, 255), -1)   # colour lost briefly
        d = Detector(cfg, bg)
        for _ in range(3):
            d.process(frame)
        targets = lambda r: [o for o in r[1] if o.stable and o.isolated]
        for _ in range(3):
            self.assertTrue(targets(d.process(whitened)))        # held
        self.assertFalse(targets(d.process(whitened)))           # 4th miss: dropped

    def test_off_by_default(self):
        cfg = pile_config()
        bg = np.full((240, 320, 3), 140, np.uint8)
        frame = bg.copy()
        cv2.rectangle(frame, (130, 100), (149, 119), color(20), -1)
        d = Detector(cfg, bg)
        for _ in range(3):
            d.process(frame)
        self.assertFalse([o for o in d.process(bg)[1] if o.stable and o.isolated])


if __name__ == '__main__':
    unittest.main()