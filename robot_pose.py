"""Robot pose from one AprilTag on top of the robot, seen by the overhead camera.

Output is in arena millimetres (same frame as vision.py targets):
x to the right, y down, origin at the top-left floor corner.
heading_deg: 0 = facing +x, 90 = facing +y (down the image), i.e. clockwise on screen.

The floor homography is exact only at floor level. The tag sits at height
tag_height_mm, so its image is pushed away from the point under the camera.
We undo this with  r_true = r_measured * (H - h) / H.

Run live to check tag size / detection rate:
    python robot_pose.py 1
    python robot_pose.py --video run.mp4
"""
import argparse
import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import cv2
import numpy as np

DICTIONARIES = {
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
}


@dataclass
class Pose:
    x: float            # tag centre on the floor plane, mm
    y: float
    heading_deg: float
    grip_x: float       # gripper point (tag centre + offset), mm
    grip_y: float
    side_mm: float      # measured tag side after correction; sanity check
    t: float

    def as_dict(self):
        return {k: round(v, 2) if isinstance(v, float) else v for k, v in asdict(self).items()}


def floor_homography(cfg):
    arena = cfg.get('arena', {})
    corners = arena.get('corners_px')
    if not corners:
        raise ValueError('arena.corners_px missing: run calibrate_arena.py first')
    w, h = arena['size_mm']
    src = np.array(corners, np.float32)
    dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], np.float32)
    return cv2.getPerspectiveTransform(src, dst)


class RobotPoseEstimator:
    def __init__(self, cfg):
        tag = cfg.get('robot_tag')
        if not tag:
            raise ValueError('robot_tag block missing in calib.json (see README_ROBOT.md)')
        arena = cfg['arena']
        self.source_size = arena.get('source_size_px')
        self.H = floor_homography(cfg)
        self.tag_id = int(tag.get('id', 0))
        self.size_mm = float(tag.get('size_mm', 100))
        self.tag_h = float(tag.get('height_mm', 0))
        self.cam_h = float(tag.get('camera_height_mm', 2000))
        if self.cam_h <= self.tag_h:
            raise ValueError('camera_height_mm must be larger than height_mm')
        w, h = arena['size_mm']
        self.cam_xy = np.array(tag.get('camera_floor_xy_mm') or [w / 2, h / 2], np.float64)
        # offset from tag centre to gripper point: [forward_mm, right_mm]
        self.offset = np.array(tag.get('grip_offset_mm', [0, 0]), np.float64)
        self.size_tolerance = float(tag.get('size_tolerance', 0.25))
        # 180 if the tag is mounted "upside down" (e.g. official apriltag-imgs PNG with its top edge forward)
        self.heading_offset = math.radians(float(tag.get('heading_offset_deg', 0)))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[tag.get('family', '36h11')])
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.last_reason = 'not run'
        self.last_corners_px = None

    def to_floor(self, pts_px):
        """Image pixels -> floor mm at tag height (parallax corrected)."""
        pts = cv2.perspectiveTransform(np.asarray(pts_px, np.float32).reshape(-1, 1, 2), self.H)
        pts = pts.reshape(-1, 2).astype(np.float64)
        return self.cam_xy + (pts - self.cam_xy) * (self.cam_h - self.tag_h) / self.cam_h

    def detect_all(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(i): c.reshape(4, 2) for i, c in zip(ids.flatten(), corners)}

    def detect(self, frame, t=None):
        t = time.time() if t is None else t
        if self.source_size and list(frame.shape[1::-1]) != list(self.source_size):
            raise ValueError('Camera resolution differs from calibration')
        tags = self.detect_all(frame)
        self.last_corners_px = tags.get(self.tag_id)
        if self.last_corners_px is None:
            self.last_reason = 'tag not seen' if not tags else f'other ids only: {sorted(tags)}'
            return None
        pts = self.to_floor(self.last_corners_px)          # TL, TR, BR, BL of the tag
        sides = np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1)
        side = float(sides.mean())
        if abs(side - self.size_mm) / self.size_mm > self.size_tolerance:
            self.last_reason = f'size {side:.0f} mm != {self.size_mm:.0f} mm (check height/calib)'
            return None
        centre = pts.mean(axis=0)
        front = (pts[0] + pts[1]) / 2                        # top edge of the printed tag = robot front
        back = (pts[2] + pts[3]) / 2
        d = front - back
        heading = math.atan2(d[1], d[0]) + self.heading_offset
        heading = math.atan2(math.sin(heading), math.cos(heading))
        forward = np.array([math.cos(heading), math.sin(heading)])
        right = np.array([-math.sin(heading), math.cos(heading)])
        grip = centre + self.offset[0] * forward + self.offset[1] * right
        self.last_reason = 'ok'
        return Pose(float(centre[0]), float(centre[1]), math.degrees(heading),
                    float(grip[0]), float(grip[1]), side, t)


def footprint_polygon_mm(pose, footprint):
    """Robot outline in arena mm. footprint = {front, back, left, right} mm from tag centre.
    Use it to mask the robot out of vision.py foreground (divide by mm_per_px)."""
    h = math.radians(pose.heading_deg)
    f = np.array([math.cos(h), math.sin(h)])
    r = np.array([-math.sin(h), math.cos(h)])
    c = np.array([pose.x, pose.y])
    return [c + f * footprint['front'] - r * footprint['left'],
            c + f * footprint['front'] + r * footprint['right'],
            c - f * footprint['back'] + r * footprint['right'],
            c - f * footprint['back'] - r * footprint['left']]


def draw(frame, estimator, pose):
    corners = estimator.last_corners_px
    if corners is not None:
        pts = corners.astype(np.int32)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        top_mid = tuple(((corners[0] + corners[1]) / 2).astype(int))
        centre = tuple(corners.mean(axis=0).astype(int))
        cv2.arrowedLine(frame, centre, top_mid, (0, 0, 255), 2, tipLength=.4)
    text = (f'x={pose.x:.0f} y={pose.y:.0f} h={pose.heading_deg:.0f} side={pose.side_mm:.0f}mm'
            if pose else estimator.last_reason)
    cv2.putText(frame, text, (10, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, .55,
                (0, 255, 0) if pose else (0, 0, 255), 2)
    return frame


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('camera_index', nargs='?', type=int)
    p.add_argument('--video')
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--headless', action='store_true')
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    est = RobotPoseEstimator(cfg)
    source = args.video or (args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0))
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open {source}')
    for name, value in cfg.get('camera_properties', {}).items():
        prop = getattr(cv2, 'CAP_PROP_' + name, None)
        if prop is None or not cap.set(prop, value):
            print(f'Warning: camera did not accept {name}={value}')
    frames = hits = 0
    last_print = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frames += 1
            pose = est.detect(frame)
            hits += pose is not None
            now = time.monotonic()
            if now - last_print > .2:
                last_print = now
                all_tags = est.detect_all(frame)
                sizes = {i: round(float(np.linalg.norm(c[0] - c[1])), 1) for i, c in all_tags.items()}
                print(f'rate={100 * hits / frames:5.1f}%  tag_px={sizes}  ',
                      pose.as_dict() if pose else est.last_reason)
            if not args.headless:
                cv2.imshow('robot_pose', draw(frame, est, pose))
                if cv2.waitKey(1) & 255 == ord('q'):
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if frames:
            print(f'detected in {hits}/{frames} frames ({100 * hits / frames:.1f}%)')


if __name__ == '__main__':
    main()
