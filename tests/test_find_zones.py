import unittest
import cv2
import numpy as np
from find_zones import find_zones, check, apply

# Field-like colors (BGR) and HSV ranges similar to a real calibration.
ZONES = {6: (107, 277, (90, 180, 110)), 3: (416, 162, (60, 60, 150)), 2: (690, 160, (150, 110, 10)),
         1: (102, 625, (80, 20, 90)), 5: (457, 727, (200, 190, 110)), 4: (756, 719, (60, 120, 200))}


def hsv_cfg():
    return {'arena': {'mm_per_px': 2}, 'zone_area_mm2': {'min': 15000, 'max': 60000},
            'hsv': {'1_violet': [{'lo': [140, 60, 45], 'hi': [175, 255, 255]}],
                    '2_cyan': [{'lo': [90, 175, 60], 'hi': [110, 255, 255]}],
                    '3_crimson': [{'lo': [0, 110, 80], 'hi': [8, 255, 255]}, {'lo': [176, 110, 80], 'hi': [179, 255, 255]}],
                    '4_orange': [{'lo': [9, 77, 100], 'hi': [25, 255, 255]}],
                    '5_skyblue': [{'lo': [85, 60, 129], 'hi': [110, 170, 255]}],
                    '6_lime': [{'lo': [40, 60, 80], 'hi': [75, 255, 255]}]}}


def field(stones=True):
    img = np.full((845, 1150, 3), (175, 200, 210), np.uint8)             # pale beige floor
    for x, y, bgr in ZONES.values():
        cv2.circle(img, (x, y), 66, (250, 250, 250), -1)                   # white ring
        cv2.circle(img, (x, y), 62, bgr, -1)
    if stones:
        rng = np.random.default_rng(0)
        for _ in range(40):
            x, y = int(rng.integers(300, 1100)), int(rng.integers(300, 600))
            cv2.rectangle(img, (x, y), (x + 14, y + 10), [int(v) for v in rng.integers(0, 255, 3)], -1)
    return img


class FindZoneTests(unittest.TestCase):
    def test_finds_six_zones_with_colors(self):
        zones, _ = find_zones(field(), hsv_cfg())
        self.assertEqual(check(zones), [])
        found = {z['color']: z for z in zones}
        for cid, (x, y, _) in ZONES.items():
            self.assertAlmostEqual(found[cid]['center_px'][0], x, delta=3)
            self.assertAlmostEqual(found[cid]['center_px'][1], y, delta=3)

    def test_missing_zone_is_reported(self):
        img = field()
        cv2.circle(img, (690, 160), 70, (175, 200, 210), -1)                 # remove one zone
        zones, _ = find_zones(img, hsv_cfg())
        self.assertTrue(any('found 5' in p for p in check(zones)))

    def test_apply_writes_grown_circles_and_centres(self):
        cfg = hsv_cfg()
        zones, _ = find_zones(field(False), cfg)
        apply(cfg, zones, ring_mm=8, margin_mm=20)
        self.assertEqual(len(cfg['exclude_polygons']), 6)
        poly = np.array(cfg['exclude_polygons'][0])
        centre = poly.mean(axis=0)
        radius = np.linalg.norm(poly - centre, axis=1).mean()
        self.assertAlmostEqual(radius, zones[0]['radius_px'] + 14, delta=2)   # (8+20) mm / 2 mm/px
        self.assertIn('center_mm', cfg['zones']['3_crimson'])


if __name__ == '__main__':
    unittest.main()