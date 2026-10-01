"""Measure robot_tag.grip_offset_mm and robot_tag.axle_offset_mm with the overhead camera.

Needs: calibrate_arena.py done, the tag on the robot, and robot_tag.height_mm,
size_mm and camera_height_mm filled in.

1) Grip offset. Close the jaws on a stone, arm DOWN (stone on the floor), robot
   anywhere in view. Use a lime, violet or cyan stone (orange can look like the roof).
       python calibrate_grip.py 1                 # shows the result
       python calibrate_grip.py 1 --write         # also saves it to calib.json
   Run it twice with the robot at different spots/headings; the two results should
   agree within ~5 mm.

2) Axle offset (how far the tag centre is ahead of the wheel axle). Start the
   script, then spin the robot slowly on the spot with teleop (a or d) for a full turn.
       python calibrate_grip.py 1 --axle --write
   The tag centre moves on a circle around the axle; its radius is the offset.

Until (1) has been saved, autonomy.py prints a warning at every start.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from robot_pose import RobotPoseEstimator
from vision import mask_for, warp


def stone_in_jaws(frame_rect, cfg, pose, per_px, stone_h, cam_xy, cam_h, color=None):
    """Centroid (mm, floor level) of the coloured blob in front of the robot, or None."""
    hsv = cv2.cvtColor(frame_rect, cv2.COLOR_BGR2HSV)
    h = math.radians(pose.heading_deg)
    f = np.array([math.cos(h), math.sin(h)])
    r = np.array([-math.sin(h), math.cos(h)])
    best = None
    for key, ranges in cfg['hsv'].items():
        if color is not None and int(str(key).split('_')[0]) != color:
            continue
        if not ranges:
            continue
        mask = mask_for(hsv, ranges)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
        for i in range(1, n):
            area_mm2 = stats[i, cv2.CC_STAT_AREA] * per_px * per_px
            if area_mm2 < 300:
                continue
            c = cents[i] * per_px
            k = (cam_h - stone_h) / cam_h                   # stone surface parallax
            c = cam_xy + (c - cam_xy) * k
            rel = c - np.array([pose.x, pose.y])
            fwd, side = float(rel @ f), float(rel @ r)
            if 85 < fwd < 350 and abs(side) < 100 and (best is None or area_mm2 > best[0]):
                best = (area_mm2, fwd, side, key)
    return best


def fit_circle(points):
    """Least-squares circle through points -> (cx, cy, radius)."""
    p = np.asarray(points, np.float64)
    A = np.column_stack([2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))])
    b = (p ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, math.sqrt(max(0.0, c + cx * cx + cy * cy))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('camera_index', nargs='?', type=int)
    ap.add_argument('--video')
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    ap.add_argument('--axle', action='store_true', help='measure axle offset (spin the robot on the spot)')
    ap.add_argument('--samples', type=int, default=60)
    ap.add_argument('--color', type=int, choices=range(1, 7), help='color ID of the stone held in the closed jaws')
    ap.add_argument('--max-spread-mm', type=float, default=10, help='refuse a noisy/ambiguous grip measurement')
    ap.add_argument('--write', action='store_true', help='save the result to the config')
    args = ap.parse_args()
    if not math.isfinite(args.max_spread_mm) or args.max_spread_mm <= 0:
        ap.error('--max-spread-mm must be positive and finite')
    cfg = json.loads(args.config.read_text())
    est = RobotPoseEstimator(cfg)
    per_px = float(cfg['arena'].get('mm_per_px', 2))
    stone_h = float(cfg.get('autonomy', {}).get('stone_height_mm', 20))
    source = args.video or (args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0))
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open {source}')
    if not args.video:
        from camera_io import apply_camera_properties
        apply_camera_properties(cap, cfg)
    grips, centres, headings = [], [], []
    need = args.samples * (3 if args.axle else 1)
    print('Axle mode: spin the robot slowly on the spot now.' if args.axle else
          'Jaws closed on a stone, arm down. Hold still.')
    while len(grips if not args.axle else centres) < need:
        ok, raw = cap.read()
        if not ok:
            break
        pose = est.detect(raw)
        view = warp(raw, cfg)
        if pose is not None:
            if args.axle:
                centres.append((pose.x, pose.y))
                headings.append(math.radians(pose.heading_deg))
            else:
                hit = stone_in_jaws(view, cfg, pose, per_px, stone_h, est.cam_xy, est.cam_h, args.color)
                if hit:
                    grips.append(hit[1:3])
                    cv2.circle(view, (round((pose.x + 0) / per_px), round(pose.y / per_px)), 4, (255, 0, 255), -1)
        done = len(centres) if args.axle else len(grips)
        cv2.putText(view, f'{done}/{need}  {est.last_reason}', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        cv2.imshow('calibrate_grip', view)
        if cv2.waitKey(1) & 255 in (ord('q'), 27):
            break
    cap.release()
    cv2.destroyAllWindows()

    tag = cfg['robot_tag']
    if args.axle:
        if len(centres) < 20:
            raise SystemExit('Not enough tag detections.')
        spread = np.ptp(np.unwrap(headings))
        if spread < math.radians(270):
            raise SystemExit(f'Only turned {math.degrees(spread):.0f} deg; spin a full turn.')
        cx, cy, radius = fit_circle(centres)
        ahead = np.median([(x - cx) * math.cos(h) + (y - cy) * math.sin(h)
                           for (x, y), h in zip(centres, headings)])
        value = round(float(math.copysign(radius, ahead)), 1)
        print(f'axle_offset_mm = {value}  (tag centre {"ahead of" if value >= 0 else "behind"} the axle)')
        if abs(value) < 8:
            print('Small enough that it hardly matters; saving it anyway.')
        tag['axle_offset_mm'] = value
    else:
        if len(grips) < 20:
            raise SystemExit('Stone not found in front of the robot often enough (see the docstring).')
        g = np.array(grips)
        fwd, side = np.median(g, axis=0)
        spread = np.percentile(np.abs(g - [fwd, side]), 90, axis=0)
        print(f'grip_offset_mm = [{fwd:.0f}, {side:.0f}]   (90% of frames within +/-{spread.max():.0f} mm)')
        if spread.max() > args.max_spread_mm:
            raise SystemExit('Measurement too variable; not saved. Hold one known-color stone still, use --color, and remove nearby stones.')
        old = tag.get('grip_offset_mm')
        if old:
            print(f'previous value: {old}')
        tag['grip_offset_mm'] = [round(float(fwd), 1), round(float(side), 1)]
        tag['grip_calibrated'] = True
    if args.write:
        args.config.write_text(json.dumps(cfg, indent=2))
        print(f'saved to {args.config}')
    else:
        print('not saved (add --write)')


if __name__ == '__main__':
    main()
