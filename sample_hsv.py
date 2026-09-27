"""Freeze a camera frame and sample real stone faces; save samples across sessions.

SPACE freezes/unfreezes. 1-6 select color; click samples only a frozen frame.
+/- changes patch radius (default 2 = 5x5 pixels). u undoes this session's last
patch for the selected color. s proposes and saves calibrated ranges; q quits.
Evaluation sessions save patches but never update the calibration.
"""
import argparse
import copy
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import uuid

import cv2
import numpy as np

from camera_io import apply_camera_properties
from color_calibration import (NAMES, ranges_from_samples, load_samples, save_json,
                               sample_signature, patches_by_color, propose_ranges,
                               sample_report, range_overlaps)
from vision import warp


def extract_patch(frame, x, y, radius, cfg):
    if not (0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]):
        raise ValueError('Click inside the camera image, above the instructions.')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    patch = hsv[max(0, y-radius):y+radius+1, max(0, x-radius):x+radius+1].reshape(-1, 3)
    reliable = ((patch[:, 1] >= cfg.get('min_saturation', 60))
                & (patch[:, 2] >= cfg.get('min_value', 45)))
    if len(patch) < 8 or np.mean(reliable) < .8:
        raise ValueError('Patch includes too much floor, shadow or white glare. Use a smaller colored face.')
    pixels = patch[reliable]
    if not ranges_from_samples(pixels, cfg.get('min_saturation', 60), cfg.get('min_value', 45)):
        raise ValueError('Patch contains mixed colors. Reduce the patch size.')
    return {'xy_px': [x, y], 'radius_px': radius, 'hsv': pixels.tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('camera_index', nargs='?', type=int)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    parser.add_argument('--samples', type=Path, help='Default: color_samples.json beside config')
    parser.add_argument('--purpose', choices=['calibration', 'evaluation'], default='calibration')
    parser.add_argument('--zoom', type=int, choices=[1, 2], default=1, help='Enlarge display; samples keep native pixel size')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    original_cfg = copy.deepcopy(cfg)
    cam = args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0)
    cfg['camera_index'] = cam
    sample_path = args.samples or args.config.with_name('color_samples.json')
    session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6]
    cap = cv2.VideoCapture(cam)
    frozen, frame, data = False, None, None
    current, radius = 1, 2
    hover = [None]
    message = 'SPACE freezes. Select 1-6, then click the colored face of a stone.'
    session = {'purpose': args.purpose, 'camera_readback': {}, 'samples': {str(c): [] for c in NAMES}}

    def click(event, x, y, flags, param):
        nonlocal message
        x, y = x // args.zoom, y // args.zoom
        if event == cv2.EVENT_MOUSEMOVE:
            hover[0] = (x, y)
            return
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not frozen:
            message = 'Press SPACE to freeze before sampling.'
            return
        try:
            patch = extract_patch(frame, x, y, radius, cfg)
            patch['captured_at'] = datetime.now(timezone.utc).isoformat()
            session['samples'][str(current)].append(patch)
            save_json(sample_path, data)
            message = f'Saved {NAMES[current]} patch. Sample different stones and positions.'
        except ValueError as e:
            message = str(e)
        print(message)

    try:
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera')
        session['camera_readback'] = apply_camera_properties(cap, cfg)
        cv2.namedWindow('sample')
        cv2.setMouseCallback('sample', click)
        print('Samples:', sample_path, '| session:', session_id, '| purpose:', args.purpose)
        while True:
            if not frozen:
                ok, raw = cap.read()
                if not ok:
                    raise RuntimeError('No camera frame')
                if data is None:
                    data = load_samples(sample_path, sample_signature(cfg, raw.shape[1::-1]))
                    data['sessions'][session_id] = session
                frame = warp(raw, cfg)
            view = frame.copy()
            if hover[0] is not None and frozen:
                x, y = hover[0]
                cv2.rectangle(view, (x-radius, y-radius), (x+radius, y+radius), (255, 255, 255), 1)
            total = len(patches_by_color(data, args.purpose)[current])
            count = len(session['samples'][str(current)])
            footer = np.full((110, max(view.shape[1], 760), 3), 30, np.uint8)
            lines = [f'{"FROZEN" if frozen else "LIVE"} | {args.purpose} | {current} {NAMES[current]} | patch {2*radius+1}x{2*radius+1}',
                     f'this session {count}, total {total} | SPACE freeze, 1-6 color, +/- size, u undo, s save, q quit',
                     message]
            for i, line in enumerate(lines):
                cv2.putText(footer, line, (8, 23+30*i), 0, .48, (240, 240, 240), 1)
            canvas = np.zeros((view.shape[0]+110, footer.shape[1], 3), np.uint8)
            canvas[:view.shape[0], :view.shape[1]] = view
            canvas[view.shape[0]:] = footer
            cv2.imshow('sample', cv2.resize(canvas, None, fx=args.zoom, fy=args.zoom, interpolation=cv2.INTER_NEAREST))
            key = cv2.waitKey(20) & 255
            if key in (ord('q'), 27):
                break
            if key == 32:
                frozen = not frozen
            if ord('1') <= key <= ord('6'):
                current = key - ord('0')
            if key in (ord('+'), ord('=')):
                radius = min(10, radius + 1)
            if key == ord('-'):
                radius = max(1, radius - 1)
            if key == ord('u') and session['samples'][str(current)]:
                session['samples'][str(current)].pop()
                save_json(sample_path, data)
                message = 'Undid the last patch for this color in this session.'
            if key == ord('s'):
                save_json(sample_path, data)
                if args.purpose == 'evaluation':
                    report = sample_report(data, cfg, 'evaluation')
                    save_json(sample_path.with_name('color_evaluation.json'), report)
                    print(json.dumps(report, indent=2))
                    message = 'Evaluation saved. Calibration was not changed.'
                    continue
                candidate, report, errors = propose_ranges(data, cfg)
                report['overlaps'] = range_overlaps(candidate)
                report['save_errors'] = errors
                save_json(sample_path.with_name('color_calibration_report.json'), report)
                if errors:
                    message = 'Not applied: ' + errors[0]
                    print('\n'.join(errors))
                    continue
                if json.loads(args.config.read_text(encoding='utf-8')) != original_cfg:
                    raise RuntimeError('Config changed in another program. Samples saved; reopen the sampler.')
                backup = args.config.with_name(args.config.stem + '.before-colors-' + session_id + '.json')
                if not backup.exists():
                    shutil.copy2(args.config, backup)
                save_json(args.config, candidate)
                cfg, original_cfg = candidate, copy.deepcopy(candidate)
                missing = [name for cid, name in NAMES.items() if not cfg['hsv'].get(f'{cid}_{name}')]
                message = 'Ranges saved. Missing colors: ' + (', '.join(missing) or 'none')
                print(message)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
