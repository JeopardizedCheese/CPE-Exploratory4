"""One camera frame -> everything the planner needs, in arena millimetres.

    snap = Perception(cfg, background).step(raw_frame, t)
    snap.pose          robot pose (robot_pose.Pose) or None; may be up to hold_s old
    snap.targets       pickable stones: [{'color', 'x', 'y', 'confidence', 'approach_deg'?}]
    snap.observations  every detection:  [{'color', 'x', 'y'}]  (color 0 = unknown)
    snap.robot_polygon robot outline that was masked out this frame (mm) or None

Robot masking: when the tag is found, the robot's footprint (robot_tag.footprint_mm
plus footprint_margin_mm) is blanked before detection, so the robot body, its roof
and the stone in its gripper produce no blobs. If the tag is missed for a frame, the
last pose keeps masking for up to hold_s seconds.

Stone parallax: vision measures on the floor plane, but a stone's visible surface is
about stone_height_mm above it, so it appears pushed away from the point under the
camera. Target positions are pulled back by the same formula robot_pose.py uses.
"""
from dataclasses import dataclass, field

import numpy as np

from robot_pose import RobotPoseEstimator, footprint_polygon_mm
from vision import Detector


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
        margin = float((tag or {}).get('footprint_margin_mm', 15))
        self.footprint = ({k: float(v) + margin for k, v in fp.items()} if fp and mask_robot else None)
        auto = cfg.get('autonomy', {})
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
            polygon = [tuple(map(float, p)) for p in footprint_polygon_mm(held, self.footprint)]
        frame, observations, _mask, status = self.detector.process(raw, [polygon] if polygon else ())
        targets, seen = [], []
        for o in observations:
            x, y = self.stone_to_floor(o.x * self.per_px, o.y * self.per_px)
            seen.append({'color': o.color, 'x': round(x, 1), 'y': round(y, 1)})
            if status == 'ok' and o.stable and o.isolated:
                target = {'color': o.color, 'x': round(x, 1), 'y': round(y, 1), 'confidence': o.confidence}
                if o.approach_deg is not None:
                    target['approach_deg'] = o.approach_deg
                targets.append(target)
        return Snapshot(t, frame, status, held, targets, seen, polygon, observations)


def draw_robot(frame, snap, per_px):
    """Outline the masked robot on the rectified frame."""
    import cv2
    if snap.robot_polygon:
        pts = np.round(np.array(snap.robot_polygon) / per_px).astype(np.int32)
        cv2.polylines(frame, [pts], True, (255, 0, 255), 2)
    if snap.pose is not None:
        c = (round(snap.pose.x / per_px), round(snap.pose.y / per_px))
        g = (round(snap.pose.grip_x / per_px), round(snap.pose.grip_y / per_px))
        cv2.arrowedLine(frame, c, g, (255, 0, 255), 2, tipLength=.25)
    return frame