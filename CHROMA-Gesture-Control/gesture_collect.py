"""Gesture trainer: record labelled hand landmarks per session (train / validation / test).

Landmarks only (no photos or video), never connects to the robot. One session = one sitting;
its split is fixed when it is created. Record every command with BOTH hands in every session.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import time
import uuid

import numpy as np

from gesture_model import (FEATURE_COUNT, FEATURE_VERSION, HANDS, LABEL_SET, LABELS, SPLITS,
                           SUGGESTED_POSE, GestureModel, newest_model)

ROOT = Path(__file__).resolve().parent
WINDOW = 'ERA-ONE gesture trainer'
MIN_SAMPLES = 30          # per command, per hand, per split: what gesture_train.py requires
PREPARE_S, SAMPLE_GAP_S = 2., .2


class Recording:
    def __init__(self, label, started, target=60, hand='R'):
        if label not in LABELS or hand not in HANDS or target < 1:
            raise ValueError('Invalid recording settings')
        self.label, self.target, self.hand = label, target, hand
        self.ready_at = started + PREPARE_S
        self.last_frame = -float('inf')
        self.samples, self.timestamps = [], []

    def add(self, hand, now):
        if (hand is None or hand.features is None or now < self.ready_at
                or not 0 <= now-hand.captured_at <= .2
                or hand.captured_at < self.ready_at
                or hand.captured_at-self.last_frame < SAMPLE_GAP_S or len(self.samples) >= self.target):
            return False
        features = np.asarray(hand.features, dtype=float)
        if features.shape != (FEATURE_COUNT,) or not np.isfinite(features).all():
            return False
        self.samples.append(features.copy())
        self.timestamps.append(hand.captured_at)
        self.last_frame = hand.captured_at
        return True

    def save(self, directory, session, split):
        if not self.samples:
            return None
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory/f'{self.label}_{self.hand}_{uuid.uuid4().hex[:12]}.npz'
        with path.open('xb') as f:
            np.savez_compressed(f, X=np.array(self.samples), label=np.array(self.label), hand=np.array(self.hand),
                                session=np.array(session), split=np.array(split),
                                feature_version=np.array(FEATURE_VERSION), label_set=np.array(LABEL_SET),
                                captured_at=np.array(self.timestamps))
        return path


def read_session(directory):
    meta = json.loads((Path(directory)/'session.json').read_text(encoding='utf-8'))
    if (meta.get('feature_version') != FEATURE_VERSION or meta.get('label_set') != LABEL_SET
            or meta.get('split') not in SPLITS):
        raise ValueError(f'Incompatible session {directory} (old gesture system or bad split)')
    return meta


def scan(data):
    """-> (Counter[(split, label, hand)] samples, Counter[split] sessions) over all sessions on disk."""
    samples, sessions = Counter(), Counter()
    for meta_path in sorted(Path(data).glob('*/session.json')):
        try:
            split = read_session(meta_path.parent)['split']
        except (ValueError, OSError, json.JSONDecodeError):
            continue
        sessions[split] += 1
        for path in meta_path.parent.glob('*.npz'):
            with np.load(path, allow_pickle=False) as d:
                samples[split, str(d['label'].item()), str(d['hand'].item())] += len(d['X'])
    return samples, sessions


def readiness(samples, sessions):
    """Per split: list of 'LABEL/hand' still short of MIN_SAMPLES (empty = ready)."""
    return {split: ([] if sessions[split] else ['no session'])
            + [f'{label}/{hand}' for label in LABELS for hand in HANDS
               if samples[split, label, hand] < MIN_SAMPLES] for split in SPLITS}


def next_missing(counts, target):
    for label in LABELS:
        for hand in ('R', 'L'):
            if counts[label, hand] < target:
                return label, hand
    return None


def render_collect(frame, hands, s, now):
    """Trainer screen (1280 x 720). s: dict with the trainer state."""
    import cv2
    from gesture_ui import (AMBER, BG, CYAN, DIM, GREEN, MUTED, RED, WHITE, bar, brand, card, draw_hand,
                            place_camera, put, put_center)
    img = np.full((720, 1280, 3), BG, np.uint8)
    split_color = {'train': GREEN, 'validation': AMBER, 'test': RED}[s['split']]
    bx = brand(img, 'GESTURE TRAINER') + 24
    cv2.rectangle(img, (bx, 12), (bx + 130, 38), split_color, -1)
    put_center(img, s['split'].upper(), bx + 65, 30, .55, (20, 20, 20), 1)
    put(img, f'session {s["session"]}   ({s["sessions"][s["split"]]} {s["split"]} session(s) on disk)'
        '   landmarks only', (bx + 146, 31), .45, MUTED)

    cx0, cy0, cw, ch = 16, 52, 800, 600
    place_camera(img, frame, cx0, cy0, cw, ch, s['error'])
    for hand in hands:
        draw_hand(img, hand, cx0, cy0, cw, ch, split_color if len(hands) == 1 else RED, HANDS.get(hand.side, '?'))
    warning = ''
    one = hands[0] if len(hands) == 1 else None
    if not hands:
        warning = 'No hand in view'
    elif len(hands) > 1:
        warning = 'Show ONE hand only (nothing is recorded with two)'
    elif one.side and one.side != s['hand']:
        warning = f'Camera sees your {HANDS[one.side]} hand; you selected {HANDS[s["hand"]]} (H switches)'
    elif not (.08 < one.x < .92 and .08 < one.y < .92):
        warning = 'Hand near the image edge: move it in'
    if warning:
        cv2.rectangle(img, (cx0, cy0), (cx0 + cw, cy0 + 34), (40, 40, 120), -1)
        put(img, warning, (cx0 + 12, cy0 + 24), .6, WHITE, 1)
    rec = s['recording']
    if rec is not None:
        left = rec.ready_at - now
        if left > 0:
            put_center(img, f'{left:.1f}', cx0 + cw/2, cy0 + 150, 3, AMBER, 6)
            put_center(img, f'GET READY: {rec.label}  ({HANDS[rec.hand]} hand)', cx0 + cw/2, cy0 + 200,
                       .8, WHITE, 2)
        else:
            cv2.circle(img, (cx0 + 30, cy0 + ch - 30), 12, RED, -1)
            put(img, f'REC {rec.label} {HANDS[rec.hand]}  {len(rec.samples)}/{rec.target}'
                '   move slowly: angle, distance, height', (cx0 + 52, cy0 + ch - 22), .6, WHITE, 2)
        bar(img, cx0, cy0 + ch - 8, cw, 8, len(rec.samples)/rec.target, RED)
    put(img, s['message'], (16, 678), .55, AMBER if s['message'].startswith('!') else WHITE)
    put(img, '1-8 command  H hand  N next missing  R record  S save  D discard  U undo  M model  ESC quit',
        (16, 706), .45, MUTED)

    px, pw = 832, 432
    label = LABELS[s['selected']]
    card(img, px, 52, pw, 124, 'SELECTED')
    put(img, f'[{s["selected"] + 1}] {label}', (px + 14, 106), .9, WHITE, 2)
    put(img, f'{HANDS[s["hand"]]} hand', (px + pw - 150, 106), .65, split_color, 2)
    put(img, 'pose: ' + SUGGESTED_POSE[label], (px + 14, 134), .45, MUTED)
    put(img, f'this session {s["counts"][label, s["hand"]]}/{s["target"]}', (px + 14, 162), .5, WHITE)

    card(img, px, 184, pw, 238, None)
    put(img, 'COMMAND', (px + 14, 204), .42, MUTED)
    put(img, 'this session', (px + 196, 204), .42, MUTED)
    put(img, f'all {s["split"]}', (px + 330, 204), .42, MUTED)
    put(img, 'L      R', (px + 200, 222), .42, MUTED)
    put(img, 'L      R', (px + 334, 222), .42, MUTED)
    for i, name in enumerate(LABELS):
        y = 246 + i*22
        if i == s['selected']:
            cv2.rectangle(img, (px + 4, y - 17), (px + pw - 4, y + 6), (70, 60, 52), -1)
        put(img, f'{i + 1} {name}', (px + 14, y), .5, WHITE)
        for col, hand in enumerate(('L', 'R')):
            n = s['counts'][name, hand]
            put(img, str(n), (px + 196 + col*50, y), .5,
                GREEN if n >= s['target'] else AMBER if n else DIM)
            total = s['totals'][s['split'], name, hand]
            put(img, str(total), (px + 330 + col*50, y), .5, GREEN if total >= MIN_SAMPLES else DIM)

    card(img, px, 430, pw, 100, f'READY TO TRAIN?   (>= {MIN_SAMPLES} per command, hand and set)')
    for i, (split, short) in enumerate(readiness(s['totals'], s['sessions']).items()):
        y = 474 + i*22
        missing = f'{len(short)} missing: ' + ', '.join(short)
        put(img, split, (px + 14, y), .5, WHITE)
        put(img, 'ready' if not short else missing[:36] + ('...' if len(missing) > 36 else ''),
            (px + 120, y), .45, GREEN if not short else MUTED)

    card(img, px, 538, pw, 174, 'MODEL CHECK  (M)')
    if not s['check']:
        put(img, 'M: live check with the newest trained model', (px + 14, 586), .48, MUTED)
        put(img, s['model_name'] or 'no model trained yet', (px + 14, 612), .42, DIM)
    elif s['model'] is None:
        put(img, 'No model yet: run gesture_train.py', (px + 14, 586), .5, AMBER)
    elif one is None:
        put(img, 'Show one hand to see what the model reads', (px + 14, 586), .5, MUTED)
    else:
        p = s['model'].probabilities(one.features)[0]
        gated, _, _ = s['model'].predict(one.features)
        put(img, f'reads: {gated}', (px + 14, 588), .7, GREEN if gated == label else AMBER, 2)
        for row, k in enumerate(np.argsort(p)[::-1][:5]):
            y = 600 + row*21
            name = str(s['model'].classes[k])
            put(img, name, (px + 14, y + 12), .45, WHITE)
            bar(img, px + 140, y, 220, 14, float(p[k]), GREEN if name == label else CYAN)
            put(img, f'{p[k]:.2f}', (px + 370, y + 12), .45, WHITE)
    return img


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--data', type=Path, default=ROOT/'gesture_data')
    parser.add_argument('--split', choices=SPLITS, default='train',
                        help='Which set this NEW session belongs to (fixed once created)')
    parser.add_argument('--samples', type=int, default=60, help='Samples per take (5 per second)')
    parser.add_argument('--note', default='', help='Who / where / lighting, saved with the session')
    parser.add_argument('--session', help='Resume a session ID printed by an earlier run')
    args = parser.parse_args(argv)
    if args.samples < MIN_SAMPLES:
        parser.error(f'Use at least {MIN_SAMPLES} samples per take (60 recommended)')
    if args.session and not re.fullmatch(r'[A-Za-z0-9_-]+', args.session):
        parser.error('Invalid session ID')
    model_path = ROOT/'models/hand_landmarker.task'
    if not model_path.is_file():
        parser.error('Hand model missing: models/hand_landmarker.task (run setup_gesture.ps1)')
    import cv2
    from hand_camera import CameraWorker

    session = args.session or time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
    directory = args.data/session
    counts = Counter()
    if args.session:
        if not (directory/'session.json').is_file():
            parser.error('Session not found; omit --session to start a new one')
        try:
            split = read_session(directory)['split']
        except ValueError as exc:
            parser.error(str(exc))
        for path in directory.glob('*.npz'):
            with np.load(path, allow_pickle=False) as data:
                counts[str(data['label'].item()), str(data['hand'].item())] += len(data['X'])
    else:
        split = args.split
        directory.mkdir(parents=True, exist_ok=False)
        (directory/'session.json').write_text(json.dumps(
            {'session': session, 'split': split, 'note': args.note, 'camera': args.camera, 'mirrored': True,
             'feature_version': FEATURE_VERSION, 'label_set': LABEL_SET, 'hands': list(HANDS),
             'created': time.strftime('%Y-%m-%d %H:%M:%S')}, indent=2), encoding='utf-8')
    totals, sessions = scan(args.data)
    newest = newest_model(ROOT/'models')
    s = {'session': session, 'split': split, 'selected': 0, 'hand': 'R', 'recording': None,
         'counts': counts, 'totals': totals, 'sessions': sessions, 'target': args.samples,
         'check': False, 'model': None, 'model_name': newest.name if newest else '', 'error': '',
         'message': 'Pick a command (1-8) and hand (H), press R; get into the pose during the countdown'}
    last_saved = None
    worker = CameraWorker(args.camera, model_path)

    def finish():
        nonlocal last_saved
        rec = s['recording']
        if rec is None:
            return
        path = rec.save(directory, session, split)
        n, key = len(rec.samples), (rec.label, rec.hand)
        if path:
            counts[key] += n
            totals[(split,) + key] += n
            last_saved = (path, key, n)
            s['message'] = f'Saved {n} samples of {rec.label} / {HANDS[rec.hand]} hand'
            follow = next_missing(counts, args.samples)
            if follow:
                s['message'] += f'.  Next missing: {follow[0]} / {HANDS[follow[1]]} (press N)'
        else:
            s['message'] = 'Nothing saved (no samples)'
        print(s['message'])
        s['recording'] = None

    print(f'Session {session} ({split}) in {directory}\nLandmarks only; no photos/video saved; no robot connection.')
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        worker.start()
        while True:
            frame, hands, error = worker.snapshot()
            s['error'] = error
            now = time.monotonic()
            rec = s['recording']
            if rec is not None:
                rec.add(hands[0] if len(hands) == 1 else None, now)
                if len(rec.samples) >= rec.target:
                    finish()
            cv2.imshow(WINDOW, render_collect(frame, hands, s, now))
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            idle = s['recording'] is None
            if ord('1') <= key <= ord('8') and idle:
                s['selected'] = key - ord('1')
            elif key in (ord('h'), ord('H')) and idle:
                s['hand'] = 'L' if s['hand'] == 'R' else 'R'
            elif key in (ord('n'), ord('N')) and idle:
                follow = next_missing(counts, args.samples)
                if follow:
                    s['selected'], s['hand'] = LABELS.index(follow[0]), follow[1]
                else:
                    s['message'] = 'This session is complete. ESC, then start a new session for more data'
            elif key in (ord('r'), ord('R')) and idle:
                s['recording'] = Recording(LABELS[s['selected']], time.monotonic(), args.samples, s['hand'])
                s['message'] = 'Recording...  S saves early, D discards'
            elif key in (ord('s'), ord('S')):
                finish()
            elif key in (ord('d'), ord('D')) and not idle:
                s['recording'] = None
                s['message'] = 'Take discarded (not saved)'
            elif key in (ord('u'), ord('U')) and idle:
                if last_saved:
                    path, k, n = last_saved
                    (directory/'discarded').mkdir(exist_ok=True)
                    path.rename(directory/'discarded'/path.name)
                    counts[k] -= n
                    totals[(split,) + k] -= n
                    s['message'] = f'Removed the last take ({k[0]} / {HANDS[k[1]]}); kept under discarded/'
                    last_saved = None
                else:
                    s['message'] = '! Nothing to undo'
            elif key in (ord('m'), ord('M')):
                s['check'] = not s['check']
                if s['check']:
                    newest = newest_model(ROOT/'models')
                    try:
                        s['model'] = GestureModel(newest) if newest else None
                        s['model_name'] = newest.name if newest else ''
                    except (OSError, ValueError, KeyError) as exc:
                        s['model'], s['message'] = None, f'! Cannot load {newest.name}: {exc}'
    finally:
        worker.end.set()
        finish()
        cv2.destroyAllWindows()
        if worker.ident is not None:
            worker.join(timeout=1.)
        print(f'Saved under {directory}\nThis session: '
              + ', '.join(f'{k[0]}/{k[1]}={n}' for k, n in sorted(counts.items()) if n))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
