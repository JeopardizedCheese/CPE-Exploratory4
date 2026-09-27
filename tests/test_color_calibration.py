import copy
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from color_calibration import (NAMES, ranges_from_samples, exclusive_labels, range_overlaps,
                               propose_ranges, sample_report, sample_signature, load_samples, save_json)
from sample_hsv import extract_patch
from find_zones import classify, check, apply
from tests.test_vision import configuration


def session(purpose, samples):
    return {'purpose': purpose, 'samples': {
        str(cid): [{'hsv': [hsv] * 25} for _ in range(5)] for cid, hsv in samples.items()}}


class ColorCalibrationTests(unittest.TestCase):
    def test_same_hue_different_saturation_can_be_separated(self):
        cfg = {'hsv': {'2_cyan': ranges_from_samples([[95, 240, 190]] * 100),
                       '5_skyblue': ranges_from_samples([[95, 105, 190]] * 100)}}
        labels = exclusive_labels(np.uint8([[[95, 240, 190], [95, 105, 190]]]), cfg)
        self.assertEqual(labels.tolist(), [[2, 5]])
        self.assertEqual(range_overlaps(cfg), [])

    def test_red_hue_wrap_classifies_both_ends(self):
        cfg = {'hsv': {'3_crimson': ranges_from_samples([[179, 200, 180]] * 30 + [[1, 200, 180]] * 30)}}
        labels = exclusive_labels(np.uint8([[[179, 200, 180], [1, 200, 180]]]), cfg)
        self.assertEqual(labels.tolist(), [[3, 3]])

    def test_ambiguous_training_samples_cannot_overwrite_calibration(self):
        cfg = {'hsv': {}}
        data = {'sessions': {'a': session('calibration', {2: [95, 200, 190], 5: [95, 200, 190]})}}
        _, report, errors = propose_ranges(data, cfg)
        self.assertTrue(errors)
        self.assertEqual(report['classes']['cyan']['ambiguous_fraction'], 1)
        self.assertEqual(cfg, {'hsv': {}})

    def test_evaluation_session_never_changes_fitted_ranges(self):
        data = {'sessions': {'a': session('calibration', {4: [20, 220, 180]})}}
        candidate, _, errors = propose_ranges(data, {'hsv': {}})
        self.assertFalse(errors)
        data['sessions']['held-out'] = session('evaluation', {4: [100, 220, 180]})
        after, _, _ = propose_ranges(data, {'hsv': {}})
        self.assertEqual(after, candidate)
        self.assertEqual(sample_report(data, after, 'evaluation')['classes']['orange']['unmatched_fraction'], 1)

    def test_too_few_patches_are_not_applied(self):
        data = {'sessions': {'a': session('calibration', {1: [150, 220, 180]})}}
        data['sessions']['a']['samples']['1'] = data['sessions']['a']['samples']['1'][:1]
        candidate, _, errors = propose_ranges(data, {'hsv': {}})
        self.assertTrue(errors)
        self.assertFalse(candidate['hsv'])

    def test_camera_geometry_change_requires_a_new_sample_file(self):
        cfg = configuration()
        signature = sample_signature(cfg, [320, 240])
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'samples.json'
            data = {'version': 1, 'signature': signature, 'sessions': {}}
            save_json(path, data)
            self.assertEqual(load_samples(path, signature), data)
            cfg['arena']['corners_px'][0] = [10, 10]
            with self.assertRaisesRegex(ValueError, 'Camera settings/geometry'):
                load_samples(path, sample_signature(cfg, [320, 240]))

    def test_patch_rejects_glare_and_footer_clicks(self):
        frame = np.full((20, 20, 3), 255, np.uint8)
        with self.assertRaises(ValueError):
            extract_patch(frame, 10, 10, 2, {})
        with self.assertRaises(ValueError):
            extract_patch(frame, 10, 25, 2, {})
        frame[:] = cv2.cvtColor(np.uint8([[[50, 200, 180]]]), cv2.COLOR_HSV2BGR)[0, 0]
        self.assertEqual(len(extract_patch(frame, 10, 10, 2, {})['hsv']), 25)

    def test_zone_ambiguity_is_unknown_and_cannot_be_saved(self):
        cfg = {'hsv': {'2_cyan': [{'lo': [80, 60, 45], 'hi': [105, 255, 255]}],
                       '5_skyblue': [{'lo': [80, 60, 45], 'hi': [105, 255, 255]}]}}
        self.assertEqual(classify([95, 220, 180], cfg), 0)
        zones = [{'color': cid} for cid in [0, 2, 3, 4, 5, 6]]
        self.assertTrue(check(zones))
        with self.assertRaises(ValueError):
            apply(cfg, zones, 8, 20)

    def test_unmatched_zone_does_not_guess_nearest_color(self):
        self.assertEqual(classify([75, 220, 180], configuration()), 0)


if __name__ == '__main__':
    unittest.main()
