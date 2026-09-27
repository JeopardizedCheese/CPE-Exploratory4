"""Keyboard control for firmware/robot_ctrl (UDP protocol v3) + run logger.

    python teleop.py 192.168.1.50                 # control only
    python teleop.py 192.168.1.50 --camera 1      # + overhead camera, video + pose logging

Keys (click the window first so it has keyboard focus):
  w/s forward/back   a/d turn in place   q/e arc left/right   SPACE stop wheels
  + / -  speed       g START (wheels enabled)   x STOP (back to IDLE, press g again)
  o/c gripper open/close    ESC quit (sends stop)

Hold mode (default): wheels stop HOLD_MS after you release a key.
--latch: a key keeps driving until SPACE (the ESP32 still stops if packets stop).

Each run is saved to runs/<time>/: video.mp4 (raw camera frames, replayable with
detect_live.py --video / robot_pose.py --video) and log.jsonl with one row per
control tick: time, frame index, action (l, r), events, ESP32 status, robot pose.
That row layout (observation + action per timestep) is what LeRobotDataset needs later.
"""
import argparse
import json
import socket
import time
import uuid
from pathlib import Path

import cv2
import numpy as np

PROTOCOL = 3


class Link:
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))
        self.sock.setblocking(False)
        self.session = uuid.uuid4().hex[:12]
        self.seq = 0
        self.status = None
        self.status_t = 0.0

    def send(self, cmd, **fields):
        self.seq += 1
        packet = {'v': PROTOCOL, 's': self.session, 'q': self.seq, 'c': cmd, **fields}
        try:
            self.sock.sendto(json.dumps(packet, separators=(',', ':')).encode(), self.addr)
        except OSError as e:
            print('send failed:', e)
        return packet

    def poll(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            except ConnectionResetError:      # Windows reports ICMP "port unreachable" this way
                continue
            except OSError:
                return
            try:
                self.status = json.loads(data)
                self.status_t = time.monotonic()
            except ValueError:
                pass

    def status_age(self):
        return time.monotonic() - self.status_t if self.status else float('inf')


DRIVE_KEYS = {
    ord('w'): (1, 1), ord('s'): (-1, -1),
    ord('a'): (-.6, .6), ord('d'): (.6, -.6),
    ord('q'): (.4, 1), ord('e'): (1, .4),
}
ARM_KEYS = {
    ord('o'): ('grip', 'open'), ord('c'): ('grip', 'close'),
}


def hud(img, lines, colors):
    for i, (text, color) in enumerate(zip(lines, colors)):
        cv2.putText(img, text, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 4)
        cv2.putText(img, text, (10, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--camera', type=int, help='overhead camera index (optional)')
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--speed', type=float, default=.5)
    p.add_argument('--rate', type=float, default=20, help='drive packets per second')
    p.add_argument('--hold-ms', type=int, default=600)
    p.add_argument('--latch', action='store_true')
    p.add_argument('--no-log', action='store_true')
    args = p.parse_args()

    link = Link(args.esp_ip, args.port)
    cfg = json.loads(args.config.read_text()) if args.config.exists() else {}

    cap = pose_est = None
    if args.camera is not None:
        cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            raise SystemExit(f'Cannot open camera {args.camera}')
        for name, value in cfg.get('camera_properties', {}).items():
            prop = getattr(cv2, 'CAP_PROP_' + name, None)
            if prop is None or not cap.set(prop, value):
                print(f'Warning: camera did not accept {name}={value}')
        if cfg.get('robot_tag') and cfg.get('arena', {}).get('corners_px'):
            from robot_pose import RobotPoseEstimator, draw
            pose_est = RobotPoseEstimator(cfg)

    run_dir = log = writer = None
    if not args.no_log:
        run_dir = Path('runs') / time.strftime('%Y%m%d-%H%M%S')
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / 'meta.json').write_text(json.dumps(
            {'session': link.session, 'esp': args.esp_ip, 'args': vars(args) | {'config': str(args.config)},
             'started': time.time(), 'calib': cfg}, indent=2, default=str))
        log = (run_dir / 'log.jsonl').open('w')
        print('logging to', run_dir)

    speed = args.speed
    drive = (0.0, 0.0)
    last_drive_key = 0.0
    last_send = 0.0
    frame_idx = -1
    events = []
    blank = np.zeros((480, 640, 3), np.uint8)
    period = 1 / args.rate

    try:
        while True:
            frame = None
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    print('camera read failed')
                    break
                frame_idx += 1
                if run_dir is not None:
                    if writer is None:
                        fps = cap.get(cv2.CAP_PROP_FPS) or 30
                        h, w = frame.shape[:2]
                        writer = cv2.VideoWriter(str(run_dir / 'video.mp4'),
                                                 cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    writer.write(frame)
            view = frame.copy() if frame is not None else blank.copy()

            key = cv2.waitKey(1 if cap is not None else 10) & 255
            now = time.monotonic()
            if key == 27:
                break
            if key in DRIVE_KEYS:
                k = DRIVE_KEYS[key]
                drive = (k[0] * speed, k[1] * speed)
                last_drive_key = now
            elif key == ord(' '):
                drive = (0.0, 0.0)
            elif key in (ord('+'), ord('=')):
                speed = min(1.0, round(speed + .1, 2))
            elif key == ord('-'):
                speed = max(.1, round(speed - .1, 2))
            elif key in ARM_KEYS:
                cmd, pos = ARM_KEYS[key]
                events.append(link.send(cmd, p=pos))
            elif key == ord('g'):
                events.append(link.send('start'))
            elif key == ord('x'):
                drive = (0.0, 0.0)
                events.append(link.send('stop'))

            if not args.latch and now - last_drive_key > args.hold_ms / 1000:
                drive = (0.0, 0.0)

            link.poll()
            pose = pose_est.detect(frame) if (pose_est is not None and frame is not None) else None

            if now - last_send >= period:
                last_send = now
                link.send('drive', l=round(drive[0], 3), r=round(drive[1], 3))
                if log is not None:
                    row = {'t': time.time(), 'frame': frame_idx,
                           'action': {'l': drive[0], 'r': drive[1]},
                           'events': [{k: e[k] for k in e if k not in ('v', 's')} for e in events],
                           'status': link.status, 'status_age_s': round(min(link.status_age(), 99), 3),
                           'pose': pose.as_dict() if pose else None}
                    log.write(json.dumps(row) + '\n')
                    events = []

            st = link.status or {}
            age = link.status_age()
            linked = age < 1.0
            state = st.get('state', '?') if linked else 'NO LINK'
            lines = [f'{state}  ({st.get("why", "")})  link {age*1000 if linked else 0:.0f}ms',
                     f'speed {speed:.1f}  cmd l={drive[0]:+.2f} r={drive[1]:+.2f}  out l={st.get("l", 0):+.2f} r={st.get("r", 0):+.2f}',
                     f'servo {st.get("servo")}  rssi {st.get("rssi")}  {"LATCH" if args.latch else "HOLD"}',
                     'wasd/qe drive  SPACE stop  g start  x stop  o/c grip  +/- speed  ESC quit']
            red, green, white = (0, 0, 255), (0, 220, 0), (230, 230, 230)
            colors = [green if state == 'RUNNING' else red, white, white, white]
            if pose_est is not None:
                draw(view, pose_est, pose)
            hud(view, lines, colors)
            cv2.imshow('teleop', view)
    finally:
        for _ in range(3):
            link.send('drive', l=0, r=0)
            link.send('stop')
            time.sleep(.02)
        if log is not None:
            log.close()
        if writer is not None:
            writer.release()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        if run_dir is not None:
            print('saved', run_dir)


if __name__ == '__main__':
    main()
