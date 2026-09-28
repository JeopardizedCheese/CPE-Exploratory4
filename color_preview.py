"""Color-only camera/image/video diagnostics. No UDP, motor or servo connection.

Examples:
  python color_preview.py --audit --config calib.json
  python color_preview.py 1
  python color_preview.py --image frame.png --background background.png --config calib.json --headless --output review

SPACE pauses a camera/video; s saves raw image, overlay, masks and JSON; q quits.
Right panel: uniquely classified pixels; white = overlapping ranges. It includes
painted zones deliberately, so color matching is visible separately from pickup.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

import cv2
import numpy as np

from camera_io import apply_camera_properties
from color_calibration import (NAMES, exclusive_labels, range_overlaps, save_json,
                               load_samples, sample_report, sample_signature)
from vision import Detector

DISPLAY_COLORS = {1: (180, 70, 180), 2: (200, 180, 0), 3: (40, 40, 230),
                  4: (0, 140, 255), 5: (255, 195, 100), 6: (60, 220, 70)}


def audit_config(cfg, samples=None):
    report = {'range_overlaps': range_overlaps(cfg),
              'missing_colors': [n for c, n in NAMES.items() if not cfg.get('hsv', {}).get(f'{c}_{n}')],
              'arena_size_mm': cfg.get('arena', {}).get('size_mm'),
              'has_corners': bool(cfg.get('arena', {}).get('corners_px'))}
    if samples is not None:
        report['calibration_fit'] = sample_report(samples, cfg, 'calibration')
        report['evaluation'] = sample_report(samples, cfg, 'evaluation')
    return report


def render(frame, observations, foreground, cfg, diagnostics, show_foreground=False):
    labels = exclusive_labels(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), cfg)
    masks = np.zeros_like(frame)
    for cid, color in DISPLAY_COLORS.items():
        masks[labels == cid] = color
    masks[labels == -1] = (255, 255, 255)
    annotated = frame.copy()
    # A stale background creates false blobs. Show marker displacement instead;
    # keep every observation in JSON for inspection.
    visible = [] if diagnostics.get('reference', {}).get('status') in ('moved', 'unverified') else observations
    visible = [o for o in visible if o.reason != 'size_out_of_range']
    for o in sorted(visible, key=lambda o: o.area, reverse=True)[:25]:
        color = DISPLAY_COLORS.get(o.color, (230, 230, 230))
        if o.bbox is not None:
            x, y, w, h = o.bbox
            cv2.rectangle(annotated, (x, y), (x+w-1, y+h-1), color, 1)
        point = (max(0, round(o.x)-15), max(12, round(o.y)))
        label = NAMES.get(o.color, '?') + ' ' + o.reason
        cv2.putText(annotated, label, point, 0, .32, (0, 0, 0), 2)
        cv2.putText(annotated, label, point, 0, .32, color, 1)
    for shift in diagnostics.get('reference', {}).get('shifts', []):
        a, b = (tuple(round(v) for v in shift[k]) for k in ('reference_px', 'current_px'))
        cv2.arrowedLine(annotated, a, b, (0, 0, 255), 2, tipLength=.2)
    h, w = frame.shape[:2]
    width = max(w*2, 900)
    canvas = np.full((h+144, width, 3), 26, np.uint8)
    canvas[42:42+h, :w] = annotated
    canvas[42:42+h, w:2*w] = cv2.cvtColor(foreground, cv2.COLOR_GRAY2BGR) if show_foreground else masks
    cv2.putText(canvas, 'COLOR PREVIEW | ' + diagnostics['status'], (10, 27), 0, .6, (255, 255, 255), 1)
    cv2.putText(canvas, 'foreground' if show_foreground else 'pixel colors | WHITE = ambiguous',
                (w+10, 27), 0, .48, (255, 255, 255), 1)
    ref = diagnostics.get('reference', {})
    counts = {NAMES[c]: sum(o.color == c for o in observations) for c in NAMES}
    lines = ['  '.join(f'{name}: {count}' for name, count in counts.items()),
             f"reference: {ref.get('status')} | markers {ref.get('matched', 0)} | "
             f"objects {len(observations)} | overlapping pixels {np.count_nonzero(labels == -1)}",
             'SPACE pause | m foreground/colors | s save evidence | q quit | no robot connection']
    if diagnostics.get('reference', {}).get('status') in ('moved', 'unverified'):
        lines[0] = 'Reference not verified. Recalibrate the empty field; inspect raw color matches in the right panel.'
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (10, h+68+i*26), 0, .45, (235, 235, 235), 1)
    return canvas, masks


def write_snapshot(folder, raw, overlay, foreground, masks, diagnostics, observations, cfg):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    for name, image in [('raw.png', raw), ('preview.png', overlay), ('foreground.png', foreground), ('colors.png', masks)]:
        if not cv2.imwrite(str(folder / name), image):
            raise OSError(f'Cannot save {name}')
    save_json(folder / 'report.json', {'diagnostics': diagnostics,
                                      'observations': [asdict(o) for o in observations]})
    save_json(folder / 'calib.json', cfg)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('camera_index', type=int, nargs='?')
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--image', type=Path, help='Raw camera image, before perspective warp')
    source.add_argument('--video', type=Path)
    parser.add_argument('--background', type=Path, help='Override the rectified empty-field reference')
    parser.add_argument('--audit', action='store_true', help='Print config/sample report without opening a camera')
    parser.add_argument('--samples', type=Path)
    parser.add_argument('--check-reference', action='store_true', help='Require fixed scoring-circle agreement')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('color_runs'))
    parser.add_argument('--max-frames', type=int, default=0, help='0 = full video / until quit')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    if args.check_reference:
        cfg.setdefault('vision', {}).setdefault('reference_guard', {})['enabled'] = True
    if args.audit:
        data = None
        if args.samples:
            data = load_samples(args.samples)
            if not data.get('signature'):
                raise SystemExit('No sample sessions recorded yet')
            data = load_samples(args.samples, sample_signature(cfg, data['signature']['source_size_px']))
        report = audit_config(cfg, data)
        print(json.dumps(report, indent=2))
        return
    background_path = args.background or args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(background_path)) if background_path.exists() else None
    detector = Detector(cfg, background)
    raw = cv2.imread(str(args.image)) if args.image else None
    if args.image and raw is None:
        raise SystemExit(f'Cannot read {args.image}')
    cap = None
    run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + uuid.uuid4().hex[:6]
    run_dir = args.output / run_id
    run_dir.mkdir(parents=True)
    save_json(run_dir / 'input.json', {'image': str(args.image) if args.image else None,
                                      'video': str(args.video) if args.video else None,
                                      'background': str(background_path), 'config': str(args.config)})
    save_json(run_dir / 'calib.json', cfg)
    paused, index, snapshots, show_foreground = False, 0, 0, False
    print('Saving evidence under', run_dir)
    try:
        if not args.image:
            source_value = str(args.video) if args.video else (args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0))
            cap = cv2.VideoCapture(source_value)
            if not cap.isOpened():
                raise RuntimeError(f'Cannot open camera/video: {source_value}')
            if not args.video:
                save_json(run_dir / 'camera_readback.json', apply_camera_properties(cap, cfg))
        with (run_dir / 'frames.jsonl').open('w', encoding='utf-8') as log:
            while True:
                if index == 0 or not paused:
                    if cap is not None:
                        ok, raw = cap.read()
                        if not ok:
                            break
                    frame, observations, foreground, status = detector.process(raw)
                    index += 1
                    log.write(json.dumps({'frame': index, **detector.diagnostics,
                                          'observations': [asdict(o) for o in observations]}) + '\n')
                    if args.image:
                        paused = True
                overlay, masks = render(frame, observations, foreground, cfg, detector.diagnostics, show_foreground)
                if args.headless:
                    if index == 1:
                        write_snapshot(run_dir / 'first-frame', raw, overlay, foreground, masks,
                                       detector.diagnostics, observations, cfg)
                    if args.image or (args.max_frames and index >= args.max_frames):
                        break
                    continue
                cv2.imshow('color preview', overlay)
                key = cv2.waitKey(20) & 255
                if key in (ord('q'), 27):
                    break
                if key == 32 and not args.image:
                    paused = not paused
                if key == ord('m'):
                    show_foreground = not show_foreground
                if key == ord('s'):
                    snapshots += 1
                    write_snapshot(run_dir / f'snapshot-{snapshots:04}', raw, overlay, foreground, masks,
                                   detector.diagnostics, observations, cfg)
                if args.max_frames and index >= args.max_frames:
                    break
        if index:
            save_json(run_dir / 'last-frame-report.json', detector.diagnostics)
            print(json.dumps(detector.diagnostics, indent=2))
        else:
            raise RuntimeError('No camera/video frames read')
    finally:
        if cap is not None:
            cap.release()
        if not args.headless:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
