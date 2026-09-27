import unittest
import cv2
import numpy as np
from perception import Perception
from tests.test_vision import configuration, color


def scene(with_robot=True):
    """320x240 px = 640x480 mm floor. Stone at (60, 60) px; robot (dark body + tag) at (200, 150) px."""
    bg = np.full((240, 320, 3), 140, np.uint8)
    frame = bg.copy()
    cv2.rectangle(frame, (50, 50), (69, 69), color(20), -1)            # orange stone
    if with_robot:
        cv2.rectangle(frame, (170, 120), (230, 180), (40, 40, 40), -1)  # robot body
        cv2.rectangle(frame, (180, 180), (220, 186), color(50), -1)     # a lime-ish part on it
        d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        tag = cv2.aruco.generateImageMarker(d, 0, 40)
        tag = cv2.copyMakeBorder(tag, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=255)
        frame[125:175, 175:225] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    return bg, frame


def config():
    cfg = configuration()
    cfg['robot_tag'] = {'id': 0, 'family': '36h11', 'size_mm': 80, 'height_mm': 0, 'camera_height_mm': 2000,
                        'grip_offset_mm': [0, 0], 'footprint_mm': {'front': 60, 'back': 60, 'left': 60, 'right': 60}}
    cfg['autonomy'] = {'stone_height_mm': 0}
    return cfg


class PerceptionTests(unittest.TestCase):
    def run_frames(self, mask_robot, n=4):
        bg, frame = scene()
        p = Perception(config(), bg, mask_robot=mask_robot)
        for i in range(n):
            snap = p.step(frame, float(i) * .1)
        return snap

    def near_robot(self, snap):
        return [o for o in snap.observations if abs(o['x'] - 400) < 90 and abs(o['y'] - 300) < 90]

    def test_robot_visible_without_mask(self):
        snap = self.run_frames(False)
        self.assertIsNotNone(snap.pose)
        self.assertTrue(self.near_robot(snap))

    def test_robot_masked_and_stone_kept(self):
        snap = self.run_frames(True)
        self.assertIsNotNone(snap.pose)
        self.assertAlmostEqual(snap.pose.x, 400, delta=4)
        self.assertEqual(self.near_robot(snap), [])
        self.assertEqual([t['color'] for t in snap.targets], [4])
        self.assertAlmostEqual(snap.targets[0]['x'], 119, delta=4)

    def test_mask_held_briefly_when_tag_missed(self):
        bg, frame = scene()
        p = Perception(config(), bg)
        p.step(frame, 0.0)
        hidden = frame.copy()
        hidden[125:175, 175:225] = 40                                    # tag not visible this frame
        self.assertIsNotNone(p.step(hidden, 0.2).robot_polygon)          # within hold_s
        self.assertIsNone(p.step(hidden, 0.9).robot_polygon)             # too old: no mask


if __name__ == '__main__':
    unittest.main()