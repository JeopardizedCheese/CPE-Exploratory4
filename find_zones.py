"""Find the six colored scoring zones in background.png and exclude them automatically.

Replaces hand-drawn exclude_polygons with circles grown by the white ring plus a
margin, and records each zone's color and center (arena mm) under "zones".
Run after calibrate_arena.py (empty field). Nothing is saved unless exactly six
zones are found and you press s (or pass --yes).

    python find_zones.py                         # preview, s = save, q = quit
    python find_zones.py --margin-mm 25
    python find_zones.py --config home/calib_home.json
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from color_calibration import NAMES, mask_for


def find_zones(background, cfg, min_saturation=None, expected=6, classify_colors=True):
    """Return (zones, threshold). Each zone: dict(center_px, radius_px, color, hsv)."""
    scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
    hsv = cv2.cvtColor(background, cv2.COLOR_BGR2HSV)
    floor_s = float(np.median(hsv[:, :, 1]))       # the pale floor dominates the image
    threshold = min_saturation if min_saturation is not None else max(55.0, floor_s + 35)
    mask = np.uint8((hsv[:, :, 1] > threshold) & (hsv[:, :, 2] > 40)) * 255
    # Remove stones/cables (small) before filling the zones' own texture gaps.
    k_open = max(3, int(round(12 / scale)) | 1)
    k_close = max(3, int(round(10 / scale)) | 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_open, k_open)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_close, k_close)))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    limits = cfg.get('zone_area_mm2', {'min': 15000, 'max': 60000})
    zones = []
    for contour in contours:
        area_px = cv2.contourArea(contour)
        area_mm2 = area_px * scale * scale
        if not (limits['min'] <= area_mm2 <= limits.get('max', 60000) * 1.4):
            continue
        (cx, cy), r = cv2.minEnclosingCircle(contour)
        fill = area_px / (np.pi * r * r)
        perimeter = cv2.arcLength(contour, True)
        circularity = 4 * np.pi * area_px / max(1.0, perimeter * perimeter)
        if fill < .7 or circularity < .7:
            continue
        # Median over the inner circle, drawn in its bounding box only (same pixels as a
        # full-image mask; the reference guard calls this every frame).
        radius = max(1, int(r * .7))
        x0, y0 = max(0, int(cx) - radius), max(0, int(cy) - radius)
        box = hsv[y0:int(cy) + radius + 1, x0:int(cx) + radius + 1]
        inside = np.zeros(box.shape[:2], np.uint8)
        cv2.circle(inside, (int(cx) - x0, int(cy) - y0), radius, 255, -1)
        median = np.median(box[inside > 0], axis=0)
        zones.append({'center_px': (float(cx), float(cy)), 'radius_px': float(r),
                      'hsv': [int(v) for v in median],
                      'color': classify(median, cfg) if classify_colors else 0})
    zones.sort(key=lambda z: (z['center_px'][1] > background.shape[0] / 2, z['center_px'][0]))
    return zones, threshold


def classify(median_hsv, cfg):
    """Only a unique calibrated match is a label. Paper colors may need manual labels."""
    pixel = np.uint8([[median_hsv]])
    hits = [int(key.split('_')[0]) for key, ranges in cfg.get('hsv', {}).items()
            if ranges and mask_for(pixel, ranges)[0, 0]]
    if len(hits) == 1:
        return hits[0]
    return 0


def circle_polygon(cx, cy, r, points=32):
    return [[int(round(cx + r * np.cos(a))), int(round(cy + r * np.sin(a)))]
            for a in np.linspace(0, 2 * np.pi, points, endpoint=False)]


def apply(cfg, zones, ring_mm, margin_mm):
    problems = check(zones)
    if problems:
        raise ValueError('; '.join(problems))
    scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
    grow = (ring_mm + margin_mm) / scale
    cfg['exclude_polygons'] = [circle_polygon(*z['center_px'], z['radius_px'] + grow) for z in zones]
    cfg['zones'] = {f'{z["color"]}_{NAMES.get(z["color"], "unknown")}':
                    {'center_mm': [round(z['center_px'][0] * scale, 1), round(z['center_px'][1] * scale, 1)],
                     'radius_mm': round(z['radius_px'] * scale, 1)} for z in zones}
    cfg['zones_margin_mm'] = {'ring': ring_mm, 'margin': margin_mm}
    return cfg


def draw(background, zones, cfg, ring_mm, margin_mm):
    scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
    view = background.copy()
    for z in zones:
        cx, cy = (int(round(v)) for v in z['center_px'])
        cv2.circle(view, (cx, cy), int(z['radius_px']), (255, 255, 255), 1)
        cv2.circle(view, (cx, cy), int(z['radius_px'] + (ring_mm + margin_mm) / scale), (0, 0, 255), 2)
        label = f'{z["color"]} {NAMES.get(z["color"], "?")}'
        cv2.putText(view, label, (cx - 40, cy), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3)
        cv2.putText(view, label, (cx - 40, cy), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
    return view


def check(zones, expected=6):
    problems = []
    if len(zones) != expected:
        problems.append(f'found {len(zones)} zones, expected {expected}')
    colors = [z['color'] for z in zones]
    if set(colors) != set(NAMES):
        problems.append('assign all six different colors; unknown/ambiguous labels cannot be saved')
    duplicate = sorted({NAMES.get(c, '?') for c in colors if colors.count(c) > 1})
    if duplicate:
        problems.append(f'same color assigned twice: {duplicate} (check HSV ranges)')
    return problems


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--margin-mm', type=float, default=20.0, help='Extra space outside the white ring')
    p.add_argument('--ring-mm', type=float, default=8.0, help='Width of the white ring around each zone')
    p.add_argument('--min-saturation', type=float, help='Override the automatic zone threshold')
    p.add_argument('--yes', action='store_true', help='Save without preview if the check passes')
    p.add_argument('--labels', type=int, nargs=6, help='Manual color IDs in the printed zone order')
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    path = args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(path))
    if background is None:
        raise SystemExit(f'No background at {path}; run calibrate_arena.py on the empty field first')
    zones, threshold = find_zones(background, cfg, args.min_saturation)
    if args.labels:
        if len(zones) != 6 or set(args.labels) != set(NAMES):
            raise SystemExit('--labels needs six detected zones and each ID 1..6 exactly once')
        for z, cid in zip(zones, args.labels):
            z['color'] = cid
    print(f'saturation threshold {threshold:.0f}')
    for index, z in enumerate(zones, 1):
        x, y = z['center_px']
        print(f'  zone {index}: {NAMES.get(z["color"], "?"):8s} center=({x:.0f},{y:.0f})px radius={z["radius_px"]:.0f}px hsv={z["hsv"]}')
    problems = check(zones)
    for problem in problems:
        print('PROBLEM:', problem)
    if args.yes:
        if problems:
            raise SystemExit('Not saved.')
        args.config.write_text(json.dumps(apply(cfg, zones, args.ring_mm, args.margin_mm), indent=2))
        print('Saved exclude_polygons and zones.')
        return
    selected = [None]
    def select(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and zones:
            i = min(range(len(zones)), key=lambda i: np.hypot(x-zones[i]['center_px'][0], y-zones[i]['center_px'][1]))
            selected[0] = i
    cv2.namedWindow('zones')
    cv2.setMouseCallback('zones', select)
    print('Click a circle, press 1 violet / 2 cyan / 3 crimson / 4 orange / 5 skyblue / 6 lime.')
    while True:
        problems = check(zones)
        view = draw(background, zones, cfg, args.ring_mm, args.margin_mm)
        if selected[0] is not None:
            z = zones[selected[0]]
            cv2.circle(view, tuple(round(v) for v in z['center_px']), round(z['radius_px'] + 4), (0, 255, 255), 3)
        hint = 'click circle, 1-6 label, s save, q quit' + (' | FIX LABELS' if problems else '')
        cv2.putText(view, hint, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 2)
        cv2.imshow('zones', view)
        key = cv2.waitKey(30) & 255
        if ord('1') <= key <= ord('6') and selected[0] is not None:
            zones[selected[0]]['color'] = key - ord('0')
        if key == ord('q') or cv2.getWindowProperty('zones', cv2.WND_PROP_VISIBLE) < 1:
            print('Not saved.')
            break
        if key == ord('s') and not problems:
            cfg = json.loads(args.config.read_text())      # reload: do not drop concurrent edits
            args.config.write_text(json.dumps(apply(cfg, zones, args.ring_mm, args.margin_mm), indent=2))
            print('Saved exclude_polygons and zones.')
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
