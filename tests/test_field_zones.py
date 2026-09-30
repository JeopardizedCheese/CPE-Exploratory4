"""Fields with fewer than six scoring zones (zone_colors) and colour aliases (color_alias)."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

import autonomy
from autonomy import Planner
from find_zones import check, expected_zone_colors, find_zones
from robot_pose import RobotPoseEstimator
import sim
from tests.test_autonomy import config, pose
from tests.test_find_zones import field, hsv_cfg


def two_zone_config():
    """Mini practice field: only crimson (3) and lime (6) zones."""
    cfg = config()
    cfg['arena']['size_mm'] = [1650, 1100]
    cfg['zones'] = {'3_crimson': {'center_mm': [300, 300], 'radius_mm': 110},
                    '6_lime': {'center_mm': [300, 800], 'radius_mm': 110}}
    cfg['zone_colors'] = [3, 6]
    cfg['hsv'] = {k: v for k, v in hsv_cfg()['hsv'].items()}
    cfg['arena']['corners_px'] = [[0, 0], [549, 0], [549, 366], [0, 366]]
    return cfg


def problems(cfg):
    with tempfile.TemporaryDirectory() as d:
        cv2.imwrite(str(Path(d) / 'background.png'), np.zeros((4, 4, 3), np.uint8))
        cfg = dict(cfg, robot_tag=cfg.get('robot_tag') or {'id': 0})
        return autonomy.setup_problems(cfg, Path(d) / 'calib.json')


class ZoneColorsSetupTests(unittest.TestCase):
    def test_default_still_requires_all_six_zones(self):
        cfg = two_zone_config()
        del cfg['zone_colors']
        self.assertTrue(any('zones' in p for p in problems(cfg)))

    def test_two_zone_field_is_ready(self):
        self.assertEqual(problems(two_zone_config()), [])

    def test_missing_listed_zone_is_reported(self):
        cfg = two_zone_config()
        del cfg['zones']['6_lime']
        self.assertTrue(any('zones' in p for p in problems(cfg)))

    def test_hsv_needed_only_for_zone_colours_and_alias_sources(self):
        cfg = two_zone_config()
        cfg['hsv'] = {k: v for k, v in cfg['hsv'].items() if k in ('3_crimson', '6_lime')}
        self.assertEqual(problems(cfg), [])
        cfg['autonomy'] = {'color_alias': {'1': 3}}
        self.assertTrue(any('HSV' in p and '1' in p for p in problems(cfg)))

    def test_unmeasured_camera_position_blocks_the_run(self):
        cfg = two_zone_config()
        cfg['robot_tag'] = {'id': 0, 'camera_height_mm': None, 'camera_floor_xy_mm': None}
        found = problems(cfg)
        self.assertTrue(any('camera_height_mm' in p for p in found))
        self.assertTrue(any('camera_floor_xy_mm' in p for p in found))
        with self.assertRaisesRegex(ValueError, 'camera_height_mm'):
            RobotPoseEstimator(cfg)

    def test_alias_must_point_at_a_zone_on_this_field(self):
        cfg = two_zone_config()
        cfg['autonomy'] = {'color_alias': {'1': 2}}
        self.assertTrue(any('color_alias' in p for p in problems(cfg)))
        cfg['autonomy'] = {'color_alias': {'1': 3, '4': 6}}
        self.assertEqual(problems(cfg), [])


class ZoneColorsFindZonesTests(unittest.TestCase):
    def test_expected_colours_default_to_all_six(self):
        self.assertEqual(expected_zone_colors({}), [1, 2, 3, 4, 5, 6])
        self.assertEqual(expected_zone_colors({'zone_colors': [6, 3]}), [3, 6])

    def test_two_zone_image_passes_the_two_zone_check_only(self):
        img = field(stones=False)
        for x, y in ((690, 160), (102, 625), (457, 727), (756, 719)):     # remove 2, 1, 5, 4
            cv2.circle(img, (x, y), 70, (175, 200, 210), -1)
        zones, _ = find_zones(img, hsv_cfg())
        self.assertEqual(sorted(z['color'] for z in zones), [3, 6])
        self.assertEqual(check(zones, [3, 6]), [])
        self.assertTrue(check(zones))                                   # six expected by default

    def test_six_zones_fail_a_two_zone_check(self):
        zones, _ = find_zones(field(stones=False), hsv_cfg())
        self.assertTrue(any('expected 2' in p for p in check(zones, [3, 6])))


class ColorAliasPlannerTests(unittest.TestCase):
    def test_colour_without_zone_is_ignored_by_default(self):
        cfg = two_zone_config()
        result = autonomy.run_sim(cfg, [sim.Stone(1000, 550, 1)], seconds=20, start=(600, 550, 0))
        self.assertNotIn('GOTO_STAGE', [row[2] for row in result['log']])
        self.assertEqual(result['robot'].stones[0].state, 'floor')

    def test_aliased_colour_is_delivered_to_the_alias_zone(self):
        cfg = two_zone_config()
        cfg['autonomy'] = {'color_alias': {'1': 3}}
        result = autonomy.run_sim(cfg, [sim.Stone(1000, 550, 1)], seconds=30, start=(600, 550, 0))
        stone = result['robot'].stones[0]
        self.assertEqual((stone.state, stone.zone), ('placed', 3))

    def test_alias_relabels_targets_and_observations_but_not_unknowns(self):
        cfg = two_zone_config()
        cfg['autonomy'] = {'color_alias': {'1': 3}}
        p = Planner(cfg)
        p.step(0.0, pose(600, 550, 0), [{'color': 1, 'x': 1000, 'y': 550, 'confidence': .8}],
               [{'color': 1, 'x': 1000, 'y': 550}, {'color': 0, 'x': 900, 'y': 300}])
        self.assertEqual(p.lock.target['color'], 3)
        self.assertEqual(p.lock.target['raw_color'], 1)

    def test_alias_can_be_switched_off_per_run(self):
        cfg = json.loads(json.dumps(two_zone_config()))
        cfg['autonomy'] = {'color_alias': {'1': 3}}
        autonomy.apply_overrides(cfg, ['color_alias={}'])
        self.assertEqual(Planner(cfg).alias, {})


if __name__ == '__main__':
    unittest.main()
