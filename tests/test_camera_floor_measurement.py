import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

import calibrate_arena
from measure_camera_floor import estimate_camera_floor, main
from vision import warp


class CameraFloorTests(unittest.TestCase):
    def test_vertical_camera_uses_optical_axis_not_field_centre(self):
        # Known perpendicular pinhole camera above (925, 650), height 1850 mm.
        field = np.array([[0, 0, 0], [2100, 0, 0], [2100, 1200, 0], [0, 1200, 0]], np.float64)
        matrix = np.array([[700, 0, 620], [0, 700, 350], [0, 0, 1]], np.float64)
        corners, _ = cv2.projectPoints(field, np.zeros(3), np.array([-925., -650., 1850.]), matrix, None)
        result = estimate_camera_floor(corners.reshape(4, 2), [2100, 1200], [1280, 720], [620, 350])
        np.testing.assert_allclose(result['robot_tag']['camera_floor_xy_mm'], [925, 650], atol=.001)
        self.assertAlmostEqual(result['measurement']['right_edge_offset_mm'], 1175, places=3)
        self.assertAlmostEqual(result['measurement']['bottom_edge_offset_mm'], 550, places=3)
        self.assertAlmostEqual(result['measurement']['origin_to_camera_diagonal_mm'], np.hypot(925, 650), places=3)
        self.assertEqual(result['measurement']['optical_center_source'], 'user_supplied')

    def test_raw_midpoint_approximation_includes_border_around_field(self):
        # The field is offset within the image. Its centre is not the raw midpoint.
        corners = [[100, 50], [800, 50], [800, 450], [100, 450]]
        result = estimate_camera_floor(corners, [2100, 1200], [1000, 600])
        np.testing.assert_allclose(result['robot_tag']['camera_floor_xy_mm'], [1198.5, 748.5], atol=.001)
        self.assertEqual(result['measurement']['optical_center_source'], 'raw_image_midpoint_approximation')
        self.assertEqual(result['measurement']['optical_center_px'], [499.5, 299.5])

    def test_floor_projection_can_be_outside_field(self):
        result = estimate_camera_floor([[600, 50], [900, 50], [900, 450], [600, 450]],
                                       [2100, 1200], [1000, 600])
        self.assertFalse(result['measurement']['inside_field'])
        self.assertLess(result['robot_tag']['camera_floor_xy_mm'][0], 0)

    def test_invalid_corners_and_dimensions_are_rejected(self):
        valid = [[100, 50], [800, 50], [800, 450], [100, 450]]
        cases = [
            (valid[:3], [2100, 1200], [1000, 600], None),
            ([valid[0], valid[2], valid[1], valid[3]], [2100, 1200], [1000, 600], None),
            (valid[::-1], [2100, 1200], [1000, 600], None),
            ([[0, 0], [1, 0], [1, 1], [0, 1]], [2100, 1200], [1000, 600], None),
            (valid, [0, 1200], [1000, 600], None),
            (valid, [2100, float('nan')], [1000, 600], None),
            (valid, [2100, 1200], [1000.5, 600], None),
            (valid, [2100, 1200], [700, 600], None),
            (valid, [2100, 1200], [1000, 600], [float('inf'), 200]),
        ]
        for case in cases:
            with self.subTest(case=case), self.assertRaises(ValueError):
                estimate_camera_floor(*case)

    def test_offline_cli_writes_report_without_touching_config_or_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / 'raw.png'
            cv2.imwrite(str(image), np.zeros((600, 1000, 3), np.uint8))
            cfg_path = root / 'calib.json'
            cfg_path.write_text(json.dumps({'arena': {'size_mm': [2100, 1200], 'source_size_px': [1000, 600],
                                                       'corners_px': [[100, 50], [800, 50], [800, 450], [100, 450]]}}))
            original = cfg_path.read_bytes()
            output = root / 'report.json'
            with patch('cv2.VideoCapture') as camera, contextlib.redirect_stdout(io.StringIO()):
                main([str(image), '--config', str(cfg_path), '--camera-downward',
                      '--use-config-corners', '--headless', '--output', str(output)])
            camera.assert_not_called()
            self.assertEqual(cfg_path.read_bytes(), original)
            np.testing.assert_allclose(json.loads(output.read_text())['robot_tag']['camera_floor_xy_mm'],
                                       [1198.5, 748.5], atol=.001)

    def test_reused_corners_require_matching_image_resolution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / 'raw.png'
            cv2.imwrite(str(image), np.zeros((30, 40, 3), np.uint8))
            cfg_path = root / 'calib.json'
            cfg_path.write_text(json.dumps({'arena': {'size_mm': [2100, 1200], 'source_size_px': [1000, 600]}}))
            with self.assertRaisesRegex(ValueError, 'resolution differs'):
                main([str(image), '--config', str(cfg_path), '--camera-downward', '--use-config-corners', '--headless'])


class ArenaCameraFloorIntegrationTests(unittest.TestCase):
    def run_calibration(self, enabled=True, save=True):
        raw = np.full((600, 1000, 3), 160, np.uint8)
        points = [[100, 50], [800, 50], [800, 450], [100, 450]]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg_path = root / 'calib.json'
            original_cfg = {'arena': {'size_mm': [2100, 1200], 'mm_per_px': 3},
                            'robot_tag': {'camera_floor_xy_mm': [111, 222], 'camera_height_mm': 1850}}
            cfg_path.write_text(json.dumps(original_cfg))
            before = cfg_path.read_bytes()
            camera = MagicMock()
            camera.isOpened.return_value = True
            camera.read.return_value = (True, raw)

            def click_corners(name, callback):
                for x, y in points:
                    callback(cv2.EVENT_LBUTTONDOWN, x, y, 0, None)

            argv = ['calibrate_arena.py', '--config', str(cfg_path)]
            if enabled:
                argv.append('--camera-downward')
            with (patch('sys.argv', argv), patch('cv2.VideoCapture', return_value=camera),
                  patch('cv2.imshow'), patch('cv2.destroyAllWindows'),
                  patch('cv2.setMouseCallback', side_effect=click_corners),
                  patch('cv2.waitKey', side_effect=[32, 13, ord('s' if save else 'q')]),
                  contextlib.redirect_stdout(io.StringIO())):
                calibrate_arena.main()
            camera.release.assert_called_once()
            if not save:
                self.assertEqual(cfg_path.read_bytes(), before)
                self.assertFalse((root / 'background.png').exists())
                return
            saved = json.loads(cfg_path.read_text())
            self.assertEqual(saved['robot_tag']['camera_height_mm'], 1850)
            expected_cfg = {'arena': {'size_mm': [2100, 1200], 'mm_per_px': 3, 'corners_px': points,
                                       'source_size_px': [1000, 600]}}
            # Preview lines/crosshair must never enter the detector's reference.
            np.testing.assert_array_equal(cv2.imread(str(root / 'background.png')), warp(raw, expected_cfg))
            return saved

    def test_saved_perpendicular_camera_measurement(self):
        saved = self.run_calibration()
        np.testing.assert_allclose(saved['robot_tag']['camera_floor_xy_mm'], [1198.5, 748.5], atol=.001)
        self.assertEqual(saved['robot_tag']['camera_floor_measurement']['optical_center_source'],
                         'raw_image_midpoint_approximation')

    def test_default_calibration_preserves_camera_floor_setting(self):
        saved = self.run_calibration(enabled=False)
        self.assertEqual(saved['robot_tag']['camera_floor_xy_mm'], [111, 222])
        self.assertNotIn('camera_floor_measurement', saved['robot_tag'])

    def test_cancelling_measurement_does_not_apply_estimate(self):
        self.run_calibration(save=False)


if __name__ == '__main__':
    unittest.main()
