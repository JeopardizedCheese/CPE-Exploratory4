import copy
import unittest
import cv2
import numpy as np
from vision import Detector, make_packet
from tests.test_vision import configuration, color

ORANGE, CYAN, VIOLET = 4, 2, 1          # test hues 20, 90, 150 in configuration()


def pile_config(on=True):
    cfg = configuration()                # 320x240 px = 640x480 mm, 2 mm/px
    cfg['vision'].update({'pile_mode': on, 'gripper_width_mm': 40, 'approach_length_mm': 60})
    return cfg


class PileTests(unittest.TestCase):
    def setUp(self):
        self.bg = np.full((240, 320, 3), 140, np.uint8)

    def run_frames(self, frame, cfg, repeats=3):
        d = Detector(cfg, self.bg)
        for _ in range(repeats):
            result = d.process(frame)
        return [o for o in result[1] if o.isolated and o.stable]

    def row_of_three(self):
        frame = self.bg.copy()
        for x0, hue in [(130, 20), (150, 90), (170, 150)]:
            cv2.rectangle(frame, (x0, 100), (x0 + 19, 119), color(hue), -1)
        return frame

    def test_off_by_default_keeps_old_behaviour(self):
        self.assertFalse(self.run_frames(self.row_of_three(), pile_config(False)))

    def test_edge_stones_pickable_from_outside(self):
        targets = {o.color: o for o in self.run_frames(self.row_of_three(), pile_config())}
        self.assertEqual(set(targets), {ORANGE, VIOLET})           # middle one is buried
        self.assertAlmostEqual(targets[ORANGE].approach_deg, 0, delta=1)    # come from the left, drive +x
        self.assertAlmostEqual(abs(targets[VIOLET].approach_deg), 180, delta=1)
        self.assertAlmostEqual(targets[ORANGE].x, 139.5, delta=1.5)

    def test_same_colour_pair_not_split(self):
        frame = self.bg.copy()
        cv2.rectangle(frame, (130, 100), (169, 119), color(20), -1)   # 80 mm long: two stones
        cv2.rectangle(frame, (170, 100), (189, 119), color(90), -1)
        colours = {o.color for o in self.run_frames(frame, pile_config())}
        self.assertNotIn(ORANGE, colours)
        self.assertIn(CYAN, colours)

    def test_single_stone_next_to_obstacle_gets_free_side(self):
        frame = self.bg.copy()
        cv2.rectangle(frame, (130, 100), (149, 119), color(20), -1)
        cv2.rectangle(frame, (157, 100), (175, 119), (255, 255, 255), -1)   # unknown thing on the right
        targets = self.run_frames(frame, pile_config())
        self.assertEqual(len(targets), 1)
        self.assertLess(abs(targets[0].approach_deg), 90)   # approach from the left half

    def test_wall_stone_approached_from_inside(self):
        frame = self.bg.copy()
        cv2.rectangle(frame, (2, 80), (21, 99), color(20), -1)
        targets = self.run_frames(frame, pile_config())
        self.assertEqual(len(targets), 1)
        self.assertGreater(abs(targets[0].approach_deg), 90)   # drive toward -x (the wall)

    def test_surrounded_stone_not_pickable(self):
        frame = self.bg.copy()
        for x0, y0, hue in [(150, 100, 20), (130, 100, 90), (170, 100, 90), (150, 80, 150), (150, 120, 150)]:
            cv2.rectangle(frame, (x0, y0), (x0 + 19, y0 + 19), color(hue), -1)
        self.assertNotIn(ORANGE, {o.color for o in self.run_frames(frame, pile_config())})

    def test_packet_carries_approach(self):
        d = Detector(pile_config(), self.bg)
        for _ in range(3):
            result = d.process(self.row_of_three())
        packet = make_packet(result[1], pile_config(), 1, '123456789abc', result[3], 0)
        self.assertTrue(packet['targets'])
        self.assertTrue(all('approach_deg' in t for t in packet['targets']))


if __name__ == '__main__':
    unittest.main()