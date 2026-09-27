"""Collect labeled hand landmarks locally; never connects to the robot."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import time
import uuid

import numpy as np

from gesture_control import CameraWorker
from gesture_model import FEATURE_VERSION, LABELS, FEATURE_COUNT


class Recording:
    def __init__(self, label, started, target=60):
        if label not in LABELS or target < 1:
            raise ValueError('Invalid recording settings')
        self.label, self.target = label, target
        self.ready_at = started+2.
        self.last_frame = -float('inf')
        self.samples, self.timestamps = [], []

    def add(self, hand, now):
        if (hand is None or hand.features is None or now < self.ready_at
                or not 0 <= now-hand.captured_at <= .2
                or hand.captured_at < self.ready_at
                or hand.captured_at-self.last_frame < .2 or len(self.samples) >= self.target):
            return False
        features = np.asarray(hand.features, dtype=float)
        if features.shape != (FEATURE_COUNT,) or not np.isfinite(features).all():
            return False
        self.samples.append(features.copy())
        self.timestamps.append(hand.captured_at)
        self.last_frame = hand.captured_at
        return True

    def save(self, directory, session):
        if not self.samples:
            return None
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory/f'{self.label}_{uuid.uuid4().hex[:12]}.npz'
        with path.open('xb') as f:
            np.savez_compressed(f, X=np.array(self.samples), label=np.array(self.label),
                                session=np.array(session), feature_version=np.array(FEATURE_VERSION),
                                captured_at=np.array(self.timestamps))
        return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0)
    parser.add_argument('--model', type=Path, default=Path(__file__).parent/'models/hand_landmarker.task')
    parser.add_argument('--data', type=Path, default=Path(__file__).parent/'gesture_data')
    parser.add_argument('--samples', type=int, default=60, help='Per take, at most 5 per second')
    parser.add_argument('--note', default='', help='Optional lighting/operator/session description')
    parser.add_argument('--session', help='Resume a session ID printed by an earlier collection run')
    args = parser.parse_args()
    if args.samples < 30:
        parser.error('Use at least 30 samples per take (60 recommended)')
    if not args.model.is_file():
        parser.error('Hand model missing; run setup_gesture.ps1')
    import cv2
    if args.session and not re.fullmatch(r'[A-Za-z0-9_-]+', args.session):
        parser.error('Invalid session ID')
    session = args.session or time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8]
    directory = args.data/session
    counts = Counter()
    if args.session:
        if not (directory/'session.json').is_file():
            parser.error('Session not found; omit --session to start a new one')
        metadata = json.loads((directory/'session.json').read_text(encoding='utf-8'))
        if metadata.get('feature_version') != FEATURE_VERSION or metadata.get('session') != session:
            parser.error('Incompatible session')
        for path in directory.glob('*.npz'):
            with np.load(path, allow_pickle=False) as data:
                if str(data['session'].item()) != session:
                    parser.error('Mixed sessions in recording directory')
                counts[str(data['label'].item())] += len(data['X'])
    else:
        directory.mkdir(parents=True, exist_ok=False)
        (directory/'session.json').write_text(json.dumps(
            {'session': session, 'note': args.note, 'camera': args.camera,
             'mirrored': True, 'feature_version': FEATURE_VERSION}, indent=2), encoding='utf-8')
    selected, recording = 0, None
    last_saved = None
    message = 'Select 1-8, press R, then prepare your pose during countdown'
    worker = CameraWorker(args.camera, args.model)
    worker.start()

    def finish():
        nonlocal recording, message, last_saved
        if recording is not None:
            path = recording.save(directory, session)
            count = len(recording.samples)
            counts[recording.label] += count
            if path:
                last_saved = (path, recording.label, count)
            message = f'Saved {count} samples of {recording.label}' if path else 'No samples saved'
            print(message)
            recording = None

    print(f'Session: {session}\nLandmarks only; no photos/video saved; no robot connection.')
    try:
        while True:
            frame, hand, error = worker.snapshot()
            now = time.monotonic()
            status = 'IDLE'
            if recording is not None:
                recording.add(hand, now)
                remaining = max(0., recording.ready_at-now)
                status = (f'PREPARE {remaining:.1f}s' if remaining else
                          f'RECORDING {len(recording.samples)}/{recording.target}')
                if len(recording.samples) >= recording.target:
                    finish()
            view = frame.copy() if frame is not None else np.zeros((480, 640, 3), dtype=np.uint8)
            lines = [f'COLLECT ONLY | {LABELS[selected]} | {status}',
                     '1 OPEN  2 FIST  3 V  4 ONE  5 THREE',
                     '6 THUMB_UP  7 THUMB_DOWN  8 UNKNOWN',
                     'R record | S save | D discard take | U undo save | ESC exit',
                     'KEEP ONE HAND VISIBLE; change angle/distance slowly',
                     error or ('Hand detected' if hand else 'No usable hand / multiple hands'),
                     message]
            lines += [f'{i+1}:{label}={counts[label]}' for i, label in enumerate(LABELS)]
            for i, text in enumerate(lines):
                cv2.putText(view, text, (8, 20+22*i), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3)
                cv2.putText(view, text, (8, 20+22*i), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            cv2.imshow('CHROMA collect training data', view)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty('CHROMA collect training data', cv2.WND_PROP_VISIBLE) < 1:
                break
            if ord('1') <= key <= ord('8') and recording is None:
                selected = key-ord('1')
            elif key == ord('r') and recording is None:
                recording = Recording(LABELS[selected], time.monotonic(), args.samples)
            elif key == ord('s'):
                finish()
            elif key == ord('d') and recording is not None:
                recording = None
                message = 'Current take discarded (not saved)'
            elif key == ord('u') and recording is None and last_saved:
                path, label, count = last_saved
                discarded = directory/'discarded'
                discarded.mkdir(exist_ok=True)
                path.rename(discarded/path.name)
                counts[label] -= count
                message = f'Excluded last {label} take; retained under discarded/'
                last_saved = None
    finally:
        worker.end.set()
        finish()
        cv2.destroyAllWindows()
        worker.join(timeout=1.)
        print(f'Saved under {directory}\nCounts: {dict(counts)}')


if __name__ == '__main__':
    main()
