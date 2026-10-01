import copy
import unittest
import cv2
import numpy as np
from vision import Detector, make_packet, _nearest_region
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


def pale(h):
    """A stone's own highlight / blurred rim: its hue, but too pale to vote for a colour."""
    return tuple(int(v) for v in cv2.cvtColor(np.uint8([[[h, 40, 225]]]), cv2.COLOR_HSV2BGR)[0, 0])


def rimmed_stone(frame, x0, y0, hue, core=14):
    """20 x 20 px (40 mm) stone: a saturated core and a pale ring, as the camera sees one."""
    cv2.rectangle(frame, (x0, y0), (x0 + 19, y0 + 19), pale(hue), -1)
    m = (20 - core) // 2
    cv2.rectangle(frame, (x0 + m, y0 + m), (x0 + m + core - 1, y0 + m + core - 1), color(hue), -1)


def v2_config(**vision):
    """pile_config() with the V2 pile switches (vision.pile_edge_pixels etc.)."""
    cfg = pile_config()
    cfg['vision'].update({'pile_edge_pixels': 'nearest'}, **vision)
    return cfg


class NearestEdgePixelTests(unittest.TestCase):
    """vision.pile_edge_pixels = "nearest": a blob pixel that votes for no colour belongs to the
    nearest single-colour region (within own_reach_mm), so a stone's pale rim no longer blocks
    its own corridor (PILE_FIX_PLAN.md)."""

    def setUp(self):
        self.bg = np.full((240, 320, 3), 140, np.uint8)

    def detect(self, frame, cfg, repeats=3):
        d = Detector(cfg, self.bg)
        for _ in range(repeats):
            result = d.process(frame)
        return result[1]

    def targets(self, frame, cfg):
        return [o for o in self.detect(frame, cfg) if o.isolated and o.stable]

    def rimmed_row(self):
        frame = self.bg.copy()
        for x0, hue in [(130, 20), (150, 90), (170, 150)]:
            rimmed_stone(frame, x0, 100, hue)
        return frame

    def test_rimmed_edge_stones_need_the_fix(self):
        frame = self.rimmed_row()
        self.assertFalse(self.targets(frame, pile_config()))         # legacy: own rim blocks the corridor
        targets = {o.color: o for o in self.targets(frame, v2_config())}
        self.assertEqual(set(targets), {ORANGE, VIOLET})               # middle one is buried
        self.assertAlmostEqual(targets[ORANGE].approach_deg, 0, delta=1)
        self.assertAlmostEqual(abs(targets[VIOLET].approach_deg), 180, delta=1)

    def test_aim_point_is_core_plus_rim(self):
        frame = self.bg.copy()
        rimmed_stone(frame, 150, 100, 90)
        rimmed_stone(frame, 170, 100, 150)
        # left stone: saturated only on its right part, pale on its outer (left) side
        cv2.rectangle(frame, (130, 100), (149, 119), pale(20), -1)
        cv2.rectangle(frame, (136, 103), (149, 116), color(20), -1)
        orange = [o for o in self.targets(frame, v2_config()) if o.color == ORANGE]
        self.assertEqual(len(orange), 1)
        self.assertAlmostEqual(orange[0].x, 139.5, delta=1.0)        # stone centre, not the core's 142.5

    def test_unvoted_pixel_goes_to_nearer_region(self):
        regions = np.zeros((5, 12), np.int32)
        regions[:, 0:3] = 2                   # numbered against row-major order on purpose
        regions[:, 9:12] = 1
        nearest, dist = _nearest_region(regions)
        self.assertTrue((nearest[:, 3:6] == 2).all())
        self.assertTrue((nearest[:, 7:9] == 1).all())
        self.assertTrue((nearest[regions > 0] == regions[regions > 0]).all())
        self.assertAlmostEqual(float(dist[2, 4]), 2.0, delta=0.1)

    def test_isolated_stone_identical_in_every_mode(self):
        frame = self.bg.copy()
        cv2.rectangle(frame, (130, 100), (149, 119), color(20), -1)
        rimmed_stone(frame, 40, 40, 90)
        def summary(cfg):
            return [(o.x, o.y, o.color, o.confidence, o.isolated, o.stable, o.approach_deg, o.reason)
                    for o in self.detect(frame, cfg)]
        legacy = summary(pile_config())
        self.assertEqual(summary(v2_config()), legacy)
        self.assertEqual(summary(v2_config(pile_outermost=True, pile_regions=True)), legacy)

    def surrounded(self):
        frame = self.bg.copy()
        for x0, y0, hue in [(150, 100, 20), (130, 100, 90), (170, 100, 90), (150, 80, 150), (150, 120, 150)]:
            rimmed_stone(frame, x0, y0, hue)
        return frame

    def test_surrounded_stone_never_a_target(self):
        cfg = v2_config(pile_outermost=True, pile_regions=True)
        self.assertNotIn(ORANGE, {o.color for o in self.targets(self.surrounded(), cfg)})

    def test_pile_regions_report_buried_stones_and_keep_the_pile_blob(self):
        observations = self.detect(self.surrounded(), v2_config(pile_regions=True))
        orange = [o for o in observations if o.color == ORANGE]
        self.assertEqual(len(orange), 1)
        self.assertFalse(orange[0].isolated)                           # seen, but not pickable
        self.assertAlmostEqual(orange[0].x, 159.5, delta=1.0)
        self.assertTrue(any(o.color == 0 for o in observations))       # the pile itself is still reported
        without = self.detect(self.surrounded(), v2_config())
        self.assertFalse([o for o in without if o.color == ORANGE])    # off: no buried stones

    def walled_row(self, x0=130):
        """Three 32 mm stones in a row with unknown stuff touching above and below along the
        whole row: no stone has a free gripper-wide corridor, but both ends have a clear exit."""
        frame = self.bg.copy()
        for i, hue in enumerate([20, 90, 150]):
            cv2.rectangle(frame, (x0 + 16 * i, 100), (x0 + 16 * i + 15, 115), color(hue), -1)
        cv2.rectangle(frame, (max(0, x0 - 10), 97), (x0 + 57, 99), (255, 255, 255), -1)
        cv2.rectangle(frame, (max(0, x0 - 10), 116), (x0 + 57, 118), (255, 255, 255), -1)
        return frame

    def test_outermost_stone_when_no_edge_stone_is_free(self):
        frame = self.walled_row()
        self.assertFalse(self.targets(frame, v2_config()))             # no free corridor anywhere
        targets = self.targets(frame, v2_config(pile_outermost=True))
        self.assertEqual(len(targets), 1)                               # one stone per pile, committed
        t = targets[0]
        self.assertIn(t.color, (ORANGE, VIOLET))                        # an end of the row
        toward_centre = 0 if t.color == ORANGE else 180                 # drive toward the pile
        self.assertAlmostEqual(abs(t.approach_deg), toward_centre, delta=1)
        self.assertTrue(any(o.color == 0 for o in self.detect(frame, v2_config(pile_outermost=True))))

    def test_outermost_corridor_still_respects_the_arena_edge(self):
        frame = self.walled_row(x0=2)                                    # row against the left wall
        targets = self.targets(frame, v2_config(pile_outermost=True))
        self.assertTrue(all(t.color != ORANGE for t in targets))     # its outside is the wall

    def test_outermost_never_picks_a_buried_stone(self):
        """When the real end stones are not stone-like (unknown neighbours claimed into them),
        the buried middle stone must not become the 'outermost' one."""
        frame = self.bg.copy()
        for x0, hue in [(130, 20), (150, 90), (170, 150)]:
            cv2.rectangle(frame, (x0, 100), (x0 + 19, 119), color(hue), -1)
        for x0, y0 in [(110, 86), (110, 114), (190, 86), (190, 114)]:
            cv2.rectangle(frame, (x0, y0), (x0 + 19, y0 + 19), (255, 255, 255), -1)
        self.assertFalse(self.targets(frame, v2_config(pile_outermost=True)))

    def test_outermost_is_not_used_when_an_edge_stone_is_free(self):
        targets = self.targets(self.rimmed_row(), v2_config(pile_outermost=True))
        self.assertEqual({o.color for o in targets}, {ORANGE, VIOLET})

    def test_legacy_mode_ignores_the_v2_switches(self):
        frame = self.walled_row()
        def summary(cfg):
            return [(o.x, o.y, o.color, o.confidence, o.isolated, o.approach_deg, o.reason)
                    for o in self.detect(frame, cfg)]
        cfg = pile_config()
        cfg['vision'].update(pile_edge_pixels='legacy', pile_outermost=True, pile_regions=True)
        self.assertEqual(summary(cfg), summary(pile_config()))


if __name__ == '__main__':
    unittest.main()