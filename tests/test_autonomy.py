import math
import unittest
import autonomy
from autonomy import Planner
from robot_pose import Pose

ZONES = {'3_crimson': (575, 165), '2_cyan': (1425, 195), '6_lime': (195, 540),
         '1_violet': (190, 1000), '5_skyblue': (870, 1050), '4_orange': (1470, 1050)}


def config(axle=0.0):
    return {'arena': {'size_mm': [2100, 1200], 'mm_per_px': 3},
            'zones': {k: {'center_mm': list(c), 'radius_mm': 130} for k, c in ZONES.items()},
            'robot_tag': {'grip_offset_mm': [120, 0], 'axle_offset_mm': axle, 'grip_calibrated': True,
                          'footprint_mm': {'front': 170, 'back': 110, 'left': 105, 'right': 105}},
            'vision': {'pile_mode': True, 'clearance_mm': 60, 'gripper_width_mm': 60, 'approach_length_mm': 80}}


NOISE = {'pose_noise_mm': 4, 'heading_noise_deg': 1.5, 'tag_dropout': 0.1, 'latency_s': 0.12, 'grip_success': 0.85}


def pose(x, y, heading, t=0.0, grip=120):
    h = math.radians(heading)
    return Pose(x, y, heading, x + grip * math.cos(h), y + grip * math.sin(h), 100, t)


class SimulatedRuns(unittest.TestCase):
    def test_scattered_clean(self):
        cfg = config()
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 0), seconds=90)
        self.assertGreaterEqual(r['correct'], 3)
        self.assertEqual(r['wrong'], 0)

    def test_noisy_never_wrong(self):
        cfg = config()
        for seed in (0, 1):
            r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', seed), seconds=90, params=NOISE, seed=seed)
            self.assertEqual(r['wrong'], 0)
            self.assertGreaterEqual(r['correct'], 2)

    def test_axle_offset_known(self):
        cfg = config(axle=40)
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 1), seconds=90, params={'axle_offset_mm': 40})
        self.assertGreaterEqual(r['correct'], 3)
        self.assertEqual(r['wrong'], 0)


class Safety(unittest.TestCase):
    def carrying_planner(self):
        p = Planner(config())
        p.state, p.since, p.carrying = 'CARRY', 0.0, 2           # carrying "cyan"
        p.pick_pos, p.pick_checked = (1000.0, 600.0), False
        return p

    def test_missed_grab_inside_other_zone_does_not_open_there(self):
        p = self.carrying_planner()
        still_there = [{'color': 2, 'x': 1000.0, 'y': 600.0}]
        inside_orange = pose(1470 - 120, 1050, 0)                # grip point at orange zone centre
        opened_in_zone = False
        for i in range(20):
            t = i * 0.1
            _, _, ev = p.step(t, pose(inside_orange.x, inside_orange.y, 0, t), [], still_there)
            opened_in_zone |= ('grip', {'p': 'open'}) in ev
        self.assertEqual(p.state, 'DISCARD')
        self.assertFalse(opened_in_zone)
        _, _, ev = p.step(3.0, pose(1000, 600, 0, 3.0), [], still_there)   # well away from zones
        self.assertIn(('grip', {'p': 'open'}), ev)

    def test_release_only_inside_own_zone(self):
        p = Planner(config())
        p.state, p.since, p.carrying, p.pick_checked = 'CARRY', 0.0, 4, True
        _, _, ev = p.step(5.0, pose(1000, 800, 0, 5.0), [], [], {'state': 'RUNNING', 'servo': [120]})
        self.assertNotIn(('grip', {'p': 'open'}), ev)
        self.assertEqual(p.state, 'CARRY')
        _, _, ev = p.step(5.1, pose(1470 - 120, 1050, 0, 5.1), [], [], {'state': 'RUNNING', 'servo': [120]})
        self.assertIn(('grip', {'p': 'open'}), ev)
        self.assertEqual(p.state, 'RELEASE')

    def test_never_sends_lift(self):
        cfg = config()
        r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', 0), seconds=30)
        self.assertGreaterEqual(r['correct'], 1)

    def test_stands_still_without_pose(self):
        p = Planner(config())
        self.assertEqual(p.step(1.0, None, [{'color': 1, 'x': 900, 'y': 600, 'confidence': .8}], [])[:2], (0.0, 0.0))
        self.assertEqual(p.step(1.0, pose(500, 500, 0, t=0.0), [], [])[:2], (0.0, 0.0))   # pose 1 s old

    def test_warning_until_calibrated(self):
        cfg = config()
        self.assertIsNone(autonomy.grip_calibration_warning(cfg))
        del cfg['robot_tag']['grip_calibrated']
        self.assertIn('calibrate_grip.py', autonomy.grip_calibration_warning(cfg))


class CircleFit(unittest.TestCase):
    def test_axle_circle(self):
        from calibrate_grip import fit_circle
        pts = [(500 + 40 * math.cos(a), 300 + 40 * math.sin(a)) for a in [i * 0.3 for i in range(21)]]
        cx, cy, r = fit_circle(pts)
        self.assertAlmostEqual(r, 40, delta=0.5)
        self.assertAlmostEqual(cx, 500, delta=0.5)


if __name__ == '__main__':
    unittest.main()