import math
import unittest
import cv2
import numpy as np
from robot_pose import RobotPoseEstimator, footprint_polygon_mm

# 640x480 image covers a 1280x960 mm floor exactly: 2 mm per pixel.
def config(height=0.0, cam=2000.0, offset=(0, 0)):
    return {'arena': {'size_mm': [1280, 960], 'mm_per_px': 2, 'source_size_px': [640, 480],
                      'corners_px': [[0, 0], [640, 0], [640, 480], [0, 480]]},
            'robot_tag': {'id': 3, 'family': '36h11', 'size_mm': 100, 'height_mm': height,
                          'camera_height_mm': cam, 'grip_offset_mm': list(offset)}}


def scene(cx, cy, rotate_cw_quarters=0, side_px=50, tag_id=3):
    img = np.full((480, 640), 200, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(d, tag_id, side_px)
    tag = np.rot90(tag, -rotate_cw_quarters).copy()
    pad = 12
    tag = cv2.copyMakeBorder(tag, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    s = tag.shape[0]
    x0, y0 = int(cx - s / 2), int(cy - s / 2)
    img[y0:y0 + s, x0:x0 + s] = tag
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


class PoseTests(unittest.TestCase):
    def test_position_and_heading_up(self):
        est = RobotPoseEstimator(config())
        pose = est.detect(scene(300, 200))
        self.assertIsNotNone(pose, est.last_reason)
        self.assertAlmostEqual(pose.x, 600, delta=4)
        self.assertAlmostEqual(pose.y, 400, delta=4)
        self.assertAlmostEqual(pose.heading_deg, -90, delta=3)   # tag top points up the image
        self.assertAlmostEqual(pose.side_mm, 100, delta=5)

    def test_rotated_heading(self):
        est = RobotPoseEstimator(config())
        pose = est.detect(scene(300, 200, rotate_cw_quarters=1))
        self.assertIsNotNone(pose, est.last_reason)
        self.assertAlmostEqual(pose.heading_deg, 0, delta=3)     # now facing +x

    def test_parallax_pulls_toward_camera(self):
        # tag at 1000 mm height under a 2000 mm camera looks twice as far from centre
        est = RobotPoseEstimator(config(height=1000, cam=2000))
        pts = est.to_floor([[640, 480]])                        # far corner in the image
        np.testing.assert_allclose(pts[0], [640 + 640 / 2, 480 + 480 / 2], atol=1e-3)

    def test_size_check_rejects_wrong_height(self):
        est = RobotPoseEstimator(config(height=1500, cam=2000))  # scales size by 0.25
        self.assertIsNone(est.detect(scene(300, 200)))
        self.assertIn('size', est.last_reason)

    def test_wrong_id_ignored(self):
        est = RobotPoseEstimator(config())
        self.assertIsNone(est.detect(scene(300, 200, tag_id=7)))

    def test_grip_offset_and_footprint(self):
        est = RobotPoseEstimator(config(offset=(120, 0)))
        pose = est.detect(scene(300, 200, rotate_cw_quarters=1))  # facing +x
        self.assertAlmostEqual(pose.grip_x, pose.x + 120, delta=4)
        poly = footprint_polygon_mm(pose, {'front': 150, 'back': 50, 'left': 100, 'right': 100})
        xs = [p[0] for p in poly]
        self.assertAlmostEqual(max(xs) - pose.x, 150, delta=4)


if __name__ == '__main__':
    unittest.main()
