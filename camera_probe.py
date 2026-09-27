"""Probe an unknown camera: what it reports, which settings it accepts, and its real fps.

    python camera_probe.py 1                       # report + 5 s fps test + live view
    python camera_probe.py 1 --try-config          # also apply calib.json camera_properties
    python camera_probe.py 1 --size 1280x720       # test another resolution

Accepted is not the same as applied: watch the image while changing light.
"""
import argparse
import json
import time
from pathlib import Path

import cv2

PROPS = ['FRAME_WIDTH', 'FRAME_HEIGHT', 'FPS', 'FOURCC', 'AUTO_EXPOSURE', 'EXPOSURE', 'GAIN',
         'BRIGHTNESS', 'CONTRAST', 'SATURATION', 'AUTO_WB', 'WB_TEMPERATURE', 'AUTOFOCUS', 'FOCUS',
         'BUFFERSIZE']


def fourcc_text(value):
    v = int(value)
    return ''.join(chr((v >> 8 * i) & 255) for i in range(4)) if v > 0 else '-'


def report(cap, title):
    print(f'--- {title} (backend {cap.getBackendName()})')
    for name in PROPS:
        value = cap.get(getattr(cv2, 'CAP_PROP_' + name))
        shown = fourcc_text(value) if name == 'FOURCC' else f'{value:g}'
        print(f'  {name:15s} {shown}')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('camera_index', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--try-config', action='store_true', help='Apply camera_properties from the config')
    p.add_argument('--size', help='Request a resolution, e.g. 1280x720')
    p.add_argument('--mjpg', action='store_true', help='Request MJPG (often needed for 30 fps above 640x480)')
    p.add_argument('--seconds', type=float, default=5)
    args = p.parse_args()

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open camera {args.camera_index}')
    report(cap, 'defaults')
    if args.mjpg:
        print('MJPG accepted' if cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')) else 'MJPG rejected')
    if args.size:
        w, h = (int(v) for v in args.size.lower().split('x'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if args.try_config:
        props = json.loads(args.config.read_text()).get('camera_properties', {})
        for name, value in props.items():
            prop = getattr(cv2, 'CAP_PROP_' + name, None)
            ok = prop is not None and cap.set(prop, value)
            print(f'  set {name}={value}: {"accepted" if ok else "REJECTED"} -> now {cap.get(prop) if prop else "n/a"}')
    if args.mjpg or args.size or args.try_config:
        report(cap, 'after changes')

    frames, shape, start = 0, None, time.monotonic()
    while time.monotonic() - start < args.seconds:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit('Camera stopped delivering frames')
        frames += 1
        shape = frame.shape
    elapsed = time.monotonic() - start
    print(f'--- measured: {frames / elapsed:.1f} fps at {shape[1]}x{shape[0]} over {elapsed:.1f} s')
    print('Live view: q quits.')
    last, fps = time.monotonic(), 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        now = time.monotonic()
        fps = .9 * fps + .1 / max(1e-3, now - last)
        last = now
        cv2.putText(frame, f'{fps:4.1f} fps  {frame.shape[1]}x{frame.shape[0]}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
        cv2.imshow('camera probe', frame)
        if cv2.waitKey(1) & 255 == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()