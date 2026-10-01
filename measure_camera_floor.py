"""Estimate the floor point under a confirmed vertically downward camera.

Click TL, TR, BR, BL of a known rectangular field in an original camera image.
The optical-axis pixel is mapped to floor mm, not the centre of the gem pile.
Without a supplied optical centre, the raw image midpoint is an approximation.
This tool reads images/configuration only; it never opens a camera or robot link.
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def estimate_camera_floor(corners_px, size_mm, source_size_px, optical_center_px=None):
    """Requires an optical axis perpendicular to the floor and consistent imagery.

    Lens distortion is not corrected here. Pass corners and optical centre from
    the same image geometry. A tilted camera needs intrinsic calibration + pose
    estimation instead; mapping its optical axis gives its aim point, not C_xy.
    """
    size = np.asarray(size_mm, np.float64)
    source = np.asarray(source_size_px, np.float64)
    if size.shape != (2,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Field size_mm must contain two positive finite dimensions')
    if (source.shape != (2,) or not np.isfinite(source).all()
            or np.any(source < 2) or np.any(source != np.round(source))):
        raise ValueError('source_size_px must contain integer image width and height >= 2')
    corners = np.asarray(corners_px, np.float32)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError('Select exactly four finite corners: TL, TR, BR, BL')
    if np.any(corners < 0) or np.any(corners > source - 1):
        raise ValueError('Corners must be inside the original image')
    if not cv2.isContourConvex(corners) or cv2.contourArea(corners, oriented=True) <= 100:
        raise ValueError('Corners must form a non-degenerate clockwise rectangle outline')
    approximate = optical_center_px is None
    centre = (source - 1) / 2 if approximate else np.asarray(optical_center_px, np.float64)
    if centre.shape != (2,) or not np.isfinite(centre).all():
        raise ValueError('optical_center_px must contain two finite pixel coordinates')
    width, height = size
    floor = np.array([[0, 0], [width, 0], [width, height], [0, height]], np.float32)
    transform = cv2.getPerspectiveTransform(corners, floor)
    point = transform @ np.array([*centre, 1.0])
    if not np.isfinite(point).all() or abs(point[2]) < 1e-12:
        raise ValueError('Optical-axis pixel does not map to a finite floor point')
    x, y = point[:2] / point[2]
    return {
        'robot_tag': {'camera_floor_xy_mm': [float(x), float(y)]},
        'measurement': {
            'method': 'vertical_optical_axis_floor_homography',
            'requires_perpendicular_camera': True,
            'optical_center_source': 'raw_image_midpoint_approximation' if approximate else 'user_supplied',
            'optical_center_px': centre.tolist(),
            'lens_distortion_corrected_by_tool': False,
            'size_mm': size.tolist(),
            'source_size_px': source.astype(int).tolist(),
            'corners_px': corners.tolist(),
            'origin_to_camera_diagonal_mm': math.hypot(x, y),
            'right_edge_offset_mm': float(width - x),
            'bottom_edge_offset_mm': float(height - y),
            'inside_field': bool(0 <= x <= width and 0 <= y <= height),
        },
    }


def draw_floor_measurement(view, result):
    """Annotate a rectified preview without changing the saved background."""
    width, height = result['measurement']['size_mm']
    x, y = result['robot_tag']['camera_floor_xy_mm']
    sx, sy = (view.shape[1] - 1) / width, (view.shape[0] - 1) / height
    end = (int(round(x * sx)), int(round(y * sy)))
    elbow = (end[0], 0)
    # Leg X, leg Y, hypotenuse; metric coordinates remain valid outside the field.
    cv2.line(view, (0, 0), elbow, (0, 200, 255), 2)
    cv2.line(view, elbow, end, (0, 200, 255), 2)
    cv2.line(view, (0, 0), end, (255, 180, 0), 2)
    cv2.drawMarker(view, end, (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
    diagonal = result['measurement']['origin_to_camera_diagonal_mm']
    labels = [f'Under camera: X={x:.1f} Y={y:.1f} mm; diagonal={diagonal:.1f} mm',
              f'Right={width-x:.1f} Bottom={height-y:.1f} mm',
              'Perpendicular camera required; raw midpoint is approximate'
              if result['measurement']['optical_center_source'] == 'raw_image_midpoint_approximation'
              else 'Perpendicular camera required; supplied optical centre']
    for i, label in enumerate(labels):
        at = (12, 28 + 24 * i)
        cv2.putText(view, label, at, cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 0, 0), 3)
        cv2.putText(view, label, at, cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
    return view


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path, help='Original full camera frame, not a warped preview')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    parser.add_argument('--camera-downward', action='store_true', required=True,
                        help='Confirm the optical axis is perpendicular to the floor')
    parser.add_argument('--optical-center', type=float, nargs=2, metavar=('CX', 'CY'))
    parser.add_argument('--use-config-corners', action='store_true',
                        help='Only for the exact camera view/resolution used in this config')
    parser.add_argument('--headless', action='store_true', help='Requires --use-config-corners')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).with_name('output') / 'camera-floor-measurement.json')
    args = parser.parse_args(argv)
    if args.headless and not args.use_config_corners:
        parser.error('--headless requires --use-config-corners')
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    raw = cv2.imread(str(args.image))
    if raw is None:
        raise ValueError(f'Cannot read original image: {args.image}')
    source_size = list(raw.shape[1::-1])
    arena = cfg['arena']
    if args.use_config_corners and source_size != arena.get('source_size_px'):
        raise ValueError('Image resolution differs from config; click fresh corners')
    points = [list(p) for p in arena.get('corners_px', [])] if args.use_config_corners else []

    def measure():
        return estimate_camera_floor(points, arena['size_mm'], source_size, args.optical_center)

    def save(result):
        result['measurement']['source_image'] = str(args.image.resolve())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(result, indent=2))
        print(f'Saved measurement: {args.output.resolve()}')
        print('Config unchanged. Check the estimate before copying camera_floor_xy_mm.')

    if args.headless:
        save(measure())
        return
    from vision import warp
    name = 'Camera floor measurement'
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, min(raw.shape[1], 1100), min(raw.shape[0], 700))
    result = None

    def click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and result is None and len(points) < 4:
            points.append([x, y])

    cv2.setMouseCallback(name, click)
    print('Click floor corners TL, TR, BR, BL. ENTER measures; u undoes; q cancels.')
    print('Camera must point vertically down. The raw image midpoint is only an optical-centre approximation.')
    try:
        while True:
            if result is None:
                view = raw.copy()
                for i, point in enumerate(points):
                    cv2.circle(view, tuple(map(int, point)), 5, (0, 0, 255), -1)
                    cv2.putText(view, ('TL', 'TR', 'BR', 'BL')[i], tuple(map(int, point)),
                                cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                centre = args.optical_center or [(source_size[0]-1)/2, (source_size[1]-1)/2]
                cv2.drawMarker(view, tuple(int(round(v)) for v in centre), (255, 180, 0),
                               cv2.MARKER_CROSS, 18, 2)
            else:
                preview_cfg = {'arena': dict(arena, corners_px=points, source_size_px=source_size)}
                view = draw_floor_measurement(warp(raw, preview_cfg), result)
            cv2.imshow(name, view)
            key = cv2.waitKey(20) & 255
            if key in (ord('q'), 27) or cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1:
                return
            if key == ord('u'):
                result = None
                if points:
                    points.pop()
            if key == 13 and result is None:
                try:
                    result = measure()
                    print('camera_floor_xy_mm:', result['robot_tag']['camera_floor_xy_mm'])
                    print('s saves a measurement report; u returns to corner selection; q cancels.')
                except ValueError as error:
                    print(error)
            if key == ord('s') and result is not None:
                save(result)
                return
    finally:
        cv2.destroyWindow(name)


if __name__ == '__main__':
    main()
