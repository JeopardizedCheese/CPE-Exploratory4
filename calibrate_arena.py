"""Capture empty arena, select corners, and exclude scoring zones/fixtures."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from find_zones import expected_zone_colors
from vision import warp
from measure_camera_floor import estimate_camera_floor, draw_floor_measurement


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('camera_index', nargs='?', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--camera-downward', action='store_true',
                   help='Estimate camera_floor_xy_mm for a confirmed perpendicular camera')
    p.add_argument('--optical-center', type=float, nargs=2, metavar=('CX', 'CY'),
                   help='Optical-axis pixel in the original frame; default midpoint is approximate')
    args = p.parse_args()
    if args.optical_center is not None and not args.camera_downward:
        p.error('--optical-center requires --camera-downward')
    cfg = json.loads(args.config.read_text())
    cam = args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0)
    cap = cv2.VideoCapture(cam)
    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        for name, value in cfg.get('camera_properties', {}).items():
            prop = getattr(cv2, 'CAP_PROP_'+name, None)
            if prop is None or not cap.set(prop, value):
                print('Camera property rejected:', name)
        print('Empty arena, same camera settings as detection. SPACE freezes; q cancels.')
        while True:
            ok, raw = cap.read()
            if not ok:
                raise RuntimeError('No camera frame')
            cv2.imshow('setup', raw)
            k = cv2.waitKey(1) & 255
            if k == ord('q'):
                return
            if k == 32:
                break
        points = []
        def click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append([x, y])
        cv2.setMouseCallback('setup', click)
        print('Click floor corners: top-left, top-right, bottom-right, bottom-left. u undo; ENTER accepts.')
        while True:
            view = raw.copy()
            for i, pt in enumerate(points):
                cv2.circle(view, tuple(pt), 5, (0, 0, 255), -1)
                cv2.putText(view, str(i+1), tuple(pt), 0, 1, (0, 0, 255), 2)
            cv2.imshow('setup', view)
            k = cv2.waitKey(10) & 255
            if k == ord('q'):
                return
            if k == ord('u') and points:
                points.pop()
            if k == 13 and len(points) == 4:
                contour = np.array(points, np.int32)
                if cv2.isContourConvex(contour) and cv2.contourArea(contour) > 100:
                    break
                print('Invalid corner order; undo and select clockwise corners.')
        cfg.setdefault('arena', {'size_mm': [2100, 1200], 'mm_per_px': 2})['corners_px'] = points.copy()
        cfg['arena']['source_size_px'] = list(raw.shape[1::-1])
        camera_floor = None
        if args.camera_downward:
            camera_floor = estimate_camera_floor(points, cfg['arena']['size_mm'],
                                                  cfg['arena']['source_size_px'], args.optical_center)
            print('Perpendicular-camera estimate:', camera_floor['robot_tag'])
            print('Optical-centre source:', camera_floor['measurement']['optical_center_source'])
            print('Lens distortion is not corrected. Review before saving.')
        background = warp(raw, cfg)
        points.clear()
        polygons = []
        print('Outline each colored zone/fixture (ENTER adds polygon; u undo), '
              'or press s right away to skip and use find_zones.py. q cancels.')
        while True:
            view = background.copy()
            if camera_floor is not None:
                draw_floor_measurement(view, camera_floor)
            for polygon in polygons:
                cv2.polylines(view, [np.array(polygon)], True, (0, 0, 255), 2)
            if points:
                cv2.polylines(view, [np.array(points)], False, (0, 255, 255), 2)
            cv2.imshow('setup', view)
            k = cv2.waitKey(10) & 255
            if k == ord('q'):
                return
            if k == ord('u'):
                if points:
                    points.pop()
                elif polygons:
                    polygons.pop()
            if k == 13 and len(points) >= 3:
                polygons.append(points.copy())
                points.clear()
            if k == ord('s'):
                if points:
                    print('Finish current polygon with ENTER or undo its points first.')
                    continue
                expected = len(expected_zone_colors(cfg))
                if 0 < len(polygons) < expected:
                    print(f'Mark all {expected} scoring zones, or none (then run find_zones.py).')
                    continue
                cfg['exclude_polygons'] = polygons
                cfg['zones'] = {}  # Destination coordinates from an old geometry are invalid.
                cfg['camera_index'] = cam
                cfg['background_path'] = 'background.png'
                if camera_floor is not None:
                    tag = cfg.setdefault('robot_tag', {})
                    tag.update(camera_floor['robot_tag'])
                    tag['camera_floor_measurement'] = camera_floor['measurement']
                if not cv2.imwrite(str(args.config.parent / 'background.png'), background):
                    raise RuntimeError('Cannot save reference')
                args.config.write_text(json.dumps(cfg, indent=2))
                print('Saved arena and reference. Recalibrate after any camera move.')
                if not polygons:
                    print('No zones drawn: run find_zones.py now to exclude them.')
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
