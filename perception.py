"""One camera frame -> everything the planner needs, in arena millimetres.

    snap = Perception(cfg, background).step(raw_frame, t)
    snap.pose          robot pose from this frame (robot_pose.Pose), or None
    snap.targets       pickable stones: [{'color', 'x', 'y', 'confidence', 'approach_deg'?}]
    snap.observations  every detection:  [{'color', 'x', 'y'}]  (color 0 = unknown)
    snap.robot_polygon robot outline that was masked out this frame (mm) or None

Robot masking: when the tag is found, the robot's footprint (robot_tag.footprint_mm
plus footprint_margin_mm) is blanked before detection, so the robot body, its roof
and the stone in its gripper produce no blobs. If the tag is missed for a frame, the
last pose keeps masking for up to hold_s seconds, but is never returned for driving.
Tags are detected in the full raw image; a tag outside the calibrated rectangle
can still provide a live pose for wall recovery.

Stone parallax: vision measures on the floor plane, but a stone's visible surface is
about stone_height_mm above it, so it appears pushed away from the point under the
camera. Target positions are pulled back by the same formula robot_pose.py uses.
"""
from dataclasses import dataclass, field
import math

import numpy as np
import cv2

from robot_pose import RobotPoseEstimator, footprint_polygon_mm
from vision import Detector
from color_calibration import color_masks
from pickup import DEFAULTS as PICKUP_DEFAULTS, jaw_error


def robot_silhouette_mm(pose, footprint, cam_xy, cam_h, body_h):
    """Image-plane footprint of the robot volume, including the elevated tag/roof.

    Detector polygons are in uncorrected floor-image coordinates. A physical
    footprint alone misses the displaced roof near the edges of an overhead view.
    """
    floor = np.asarray(footprint_polygon_mm(pose, footprint), np.float32)
    top = cam_xy+(floor-cam_xy)*cam_h/(cam_h-body_h)
    return cv2.convexHull(np.float32(np.vstack([floor, top]))).reshape(-1, 2).tolist()


@dataclass
class Snapshot:
    t: float
    frame: object
    status: str
    pose: object = None
    targets: list = field(default_factory=list)
    observations: list = field(default_factory=list)
    robot_polygon: list = None
    raw_observations: list = field(default_factory=list)   # vision.Observation, rectified px
    jaw_observations: list = field(default_factory=list)   # fresh, local colour evidence only
    capture_polygon: list = None


class Perception:
    def __init__(self, cfg, background, hold_s=0.3, mask_robot=True):
        self.cfg = cfg
        self.detector = Detector(cfg, background)
        arena = cfg.get('arena', {})
        self.per_px = float(arena.get('mm_per_px', 2))
        tag = cfg.get('robot_tag')
        self.pose_est = RobotPoseEstimator(cfg) if tag and arena.get('corners_px') else None
        self.hold_s = hold_s
        self.last_pose = None
        fp = (tag or {}).get('footprint_mm')
        margin = float((tag or {}).get('footprint_margin_mm', 15)) + float(cfg.get('vision', {}).get('robot_mask_padding_mm', 20))
        self.footprint = ({k: float(v) + margin for k, v in fp.items()} if fp and mask_robot else None)
        if self.footprint:
            # Include the open fingers in the image mask. Jaw stones are detected
            # separately below, so masking the fingers does not hide pickup evidence.
            reach = float((tag or {}).get('grip_offset_mm', [0, 0])[0])
            self.footprint['front'] = max(self.footprint['front'], reach+35)
        auto = cfg.get('autonomy', {})
        self.pickup_options = dict({**PICKUP_DEFAULTS, 'approach_max_side_mm': 18}, **auto)
        self.stone_h = float(auto.get('stone_height_mm', 20))
        if self.pose_est is not None:
            self.cam_xy, self.cam_h = self.pose_est.cam_xy, self.pose_est.cam_h
        else:
            self.cam_xy, self.cam_h = None, None

    def stone_to_floor(self, x, y):
        if self.cam_xy is None or not self.stone_h:
            return x, y
        k = (self.cam_h - self.stone_h) / self.cam_h
        return (float(self.cam_xy[0] + (x - self.cam_xy[0]) * k),
                float(self.cam_xy[1] + (y - self.cam_xy[1]) * k))

    def step(self, raw, t):
        pose = self.pose_est.detect(raw, t) if self.pose_est else None
        if pose is not None:
            self.last_pose = pose
        held = self.last_pose if self.last_pose is not None and t - self.last_pose.t <= self.hold_s else None
        polygon = None
        if held is not None and self.footprint:
            polygon = robot_silhouette_mm(held, self.footprint, self.cam_xy, self.cam_h, self.pose_est.tag_h)
        frame, observations, _mask, status = self.detector.process(raw, [polygon] if polygon else ())
        targets, seen = [], []
        for o in observations:
            x, y = self.stone_to_floor(o.x * self.per_px, o.y * self.per_px)
            if not o.held:   # sticky target ghosts are not fresh obstacle/jaw evidence
                seen.append({'color': o.color, 'x': round(x, 1), 'y': round(y, 1),
                             'radius_mm': round(o.radius_mm, 1) if o.radius_mm else 0.0})
            if status == 'ok' and o.stable and o.isolated:
                target = {'color': o.color, 'x': round(x, 1), 'y': round(y, 1),
                          'confidence': o.confidence,
                          'radius_mm': round(o.radius_mm, 1) if o.radius_mm else 0.0}
                if o.approach_deg is not None:
                    target['approach_deg'] = o.approach_deg
                if o.approach_options:
                    target['approach_options'] = list(o.approach_options)
                targets.append(target)
        jaws = self._jaw_observations(frame, pose) if status == 'ok' and pose else []
        capture = None
        if pose:
            a = math.radians(pose.heading_deg)
            o = self.pickup_options
            capture = [(pose.grip_x+f*math.cos(a)-r*math.sin(a), pose.grip_y+f*math.sin(a)+r*math.cos(a))
                       for f, r in [(-o['grip_capture_back_mm'], -o['approach_max_side_mm']),
                                    (o['grip_capture_front_mm'], -o['approach_max_side_mm']),
                                    (o['grip_capture_front_mm'], o['approach_max_side_mm']),
                                    (-o['grip_capture_back_mm'], o['approach_max_side_mm'])]]
        # Keep jaw candidates separate: a held stone must not block wall recovery.
        return Snapshot(t, frame, status, pose, targets, seen, polygon, observations, jaws, capture)

    def _jaw_observations(self, frame, pose):
        """Recognize coloured stones in the jaws, without the body/edge target mask.

        Does not create navigation targets or bypass reference/lighting checks.
        Scoring-zone exclusions still apply. Mixed/ambiguous colours remain unknown.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = color_masks(hsv, self.cfg)
        ownership = sum((m > 0).astype(np.uint8) for m in masks.values()) if masks else np.zeros(frame.shape[:2], np.uint8)
        valid = np.ones(frame.shape[:2], np.uint8)*255
        for polygon in self.cfg.get('exclude_polygons', []):
            cv2.fillPoly(valid, [np.asarray(polygon, np.int32)], 0)
        result = []
        for color, mask in masks.items():
            pixels = np.uint8((mask > 0) & (ownership == 1) & (valid > 0))*255
            n, _, stats, centers = cv2.connectedComponentsWithStats(pixels)
            for i in range(1, n):
                area = stats[i, cv2.CC_STAT_AREA]*self.per_px**2
                if not 150 <= area <= self.cfg.get('gem_area_mm2', {}).get('max', 4000):
                    continue
                x, y = self.stone_to_floor(*(centers[i]*self.per_px))
                ob = {'x': round(x, 1), 'y': round(y, 1), 'color': color, 'source': 'jaw_camera'}
                along, side = jaw_error(pose, ob)
                o = self.pickup_options
                if -o['grip_capture_back_mm'] <= along <= o['grip_capture_front_mm'] and abs(side) <= o['approach_max_side_mm']:
                    result.append(ob)
        return result


def draw_robot(frame, snap, per_px):
    """Outline the masked robot on the rectified frame."""
    import cv2
    if snap.robot_polygon:
        pts = np.round(np.array(snap.robot_polygon) / per_px).astype(np.int32)
        cv2.polylines(frame, [pts], True, (255, 0, 255), 2)
    if snap.capture_polygon:
        cv2.polylines(frame, [np.round(np.array(snap.capture_polygon)/per_px).astype(np.int32)], True, (0, 220, 0), 1)
    for ob in snap.jaw_observations:
        cv2.circle(frame, (round(ob['x']/per_px), round(ob['y']/per_px)), 4, (0, 220, 0), 2)
    if snap.pose is not None:
        c = (round(snap.pose.x / per_px), round(snap.pose.y / per_px))
        g = (round(snap.pose.grip_x / per_px), round(snap.pose.grip_y / per_px))
        cv2.arrowedLine(frame, c, g, (255, 0, 255), 2, tipLength=.25)
    return frame
