import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2

from color_preview import render, write_snapshot
from vision import Detector


ROOT = Path(__file__).resolve().parents[1]


class ProvidedPhotoTests(unittest.TestCase):
    def test_mismatched_real_pair_is_blocked_and_evidence_is_serializable(self):
        cfg = json.loads((ROOT / 'examples' / 'historical_calib.json').read_text())
        cfg['vision']['reference_guard'] = {'enabled': True}
        raw = cv2.imread(str(ROOT / 'examples' / 'image.png'))
        background = cv2.imread(str(ROOT / 'examples' / 'image1.png'))
        self.assertIsNotNone(raw)
        self.assertIsNotNone(background)
        detector = Detector(cfg, background)
        frame, objects, foreground, status = detector.process(raw)
        self.assertEqual(status, 'reference_moved')
        self.assertFalse(any(o.stable for o in objects))
        self.assertGreaterEqual(detector.diagnostics['reference']['matched'], 3)
        pairs = [r['names'] for r in detector.diagnostics['range_overlaps']]
        self.assertIn(['cyan', 'skyblue'], pairs)
        self.assertIn(['crimson', 'orange'], pairs)
        overlay, masks = render(frame, objects, foreground, cfg, detector.diagnostics)
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d) / 'evidence'
            write_snapshot(folder, raw, overlay, foreground, masks, detector.diagnostics, objects, cfg)
            saved = json.loads((folder / 'report.json').read_text())
            self.assertEqual(saved['diagnostics']['status'], 'reference_moved')
            self.assertEqual(cv2.imread(str(folder / 'raw.png')).shape, raw.shape)

    def test_new_measured_profile_requires_calibration(self):
        # Checks the setup gate, not the file's current state: the team fills in
        # corners, colors and zones over time, so they are cleared here.
        cfg = json.loads((ROOT / 'field' / 'calib.json').read_text())
        self.assertEqual(cfg['arena']['size_mm'], [2100, 1200])
        self.assertGreater(cfg['robot_tag']['camera_height_mm'], 0)
        cfg['hsv'] = {}
        raw = cv2.imread(str(ROOT / 'examples' / 'image.png'))
        detector = Detector(cfg)
        _, objects, _, status = detector.process(raw)
        self.assertEqual(status, 'setup_required')
        self.assertEqual(len(detector.diagnostics['missing_colors']), 6)
        self.assertEqual(objects, [])


if __name__ == '__main__':
    unittest.main()
