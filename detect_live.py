"""Camera or recorded-video detection; see README.md for calibration and protocol."""
import argparse
import json
from pathlib import Path
import socket
import threading
import time
import uuid
import cv2
import numpy as np
from vision import make_packet
from perception import Perception, draw_robot


class LatestFrame:
    """Reads the camera in a background thread and keeps only the newest frame.

    Without this, frames queue up in the driver while a slow frame is being
    processed, and the display/targets fall further and further behind."""

    def __init__(self, cap):
        self.cap, self.frame, self.ok = cap, None, True
        self.new = threading.Condition()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.ok:
            ok, frame = self.cap.read()
            with self.new:
                self.ok, self.frame = ok, frame if ok else None
                self.new.notify_all()

    def read(self, timeout=2.0):
        with self.new:
            if self.frame is None and self.ok:
                self.new.wait(timeout)
            frame, self.frame = self.frame, None
            return frame is not None, frame

    def stop(self):
        self.ok = False
        self.thread.join(timeout=1.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('camera_index', nargs='?', type=int)
    parser.add_argument('esp_ip', nargs='?', default='127.0.0.1')
    parser.add_argument('esp_port', nargs='?', type=int, default=4210)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    parser.add_argument('--video', help='Recorded video; UDP disabled for replay')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--debug', action='store_true', help='Print one line per blob (slow)')
    parser.add_argument('--no-robot-mask', action='store_true', help='Do not hide the robot (tag) from detection')
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    if args.debug:
        cfg.setdefault('vision', {})['debug_blobs'] = True
    background_path = args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(background_path)) if background_path.exists() else None
    perception = Perception(cfg, background, mask_robot=not args.no_robot_mask)
    per_px = float(cfg.get('arena', {}).get('mm_per_px', 2))
    source = args.video or (args.camera_index if args.camera_index is not None else cfg.get('camera_index', 0))
    cap = cv2.VideoCapture(source)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session = uuid.uuid4().hex[:12]
    seq = 0
    last_send = 0
    def send(observations, status):
        nonlocal seq
        seq += 1
        packet = make_packet(observations, cfg, seq, session, status, time.time())
        if not args.video:
            sock.sendto(json.dumps(packet, separators=(',', ':')).encode(), (args.esp_ip, args.esp_port))
        return packet
    try:
        if not cap.isOpened():
            raise RuntimeError(f'Cannot open camera/video: {source}')
        for name, value in cfg.get('camera_properties', {}).items():
            prop = getattr(cv2, 'CAP_PROP_'+name, None)
            if prop is None or not cap.set(prop, value):
                print(f'Warning: camera did not accept {name}={value}')
        if not args.video:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            reader = LatestFrame(cap)
        fps, last_frame_t = 0.0, time.monotonic()
        while True:
            ok, raw = reader.read() if not args.video else cap.read()
            if not ok:
                break
            started = time.perf_counter()
            snap = perception.step(raw, time.monotonic())
            frame, observations, status = snap.frame, snap.raw_observations, snap.status
            process_ms = (time.perf_counter() - started) * 1000
            t = time.monotonic()
            fps = .9 * fps + .1 / max(1e-3, t - last_frame_t)
            last_frame_t = t
            now = time.monotonic()
            if now-last_send >= .1:
                packet = send(observations, status)
                last_send = now
                if args.headless:
                    print(json.dumps(packet))
            if not args.headless:
                for o in observations:
                    eligible = o.stable and o.isolated
                    color = (0, 220, 0) if eligible else (0, 180, 255)
                    point = (round(o.x), round(o.y))
                    cv2.circle(frame, point, 12, color, 2)
                    if o.approach_deg is not None:   # pile mode: arrow = robot's driving direction
                        a = np.radians(o.approach_deg)
                        tail = (round(o.x - 35*np.cos(a)), round(o.y - 35*np.sin(a)))
                        cv2.arrowedLine(frame, tail, point, color, 2, tipLength=.3)
                    cv2.putText(frame, f'{o.color or "?"} {o.confidence:.2f}', point,
                                cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
                cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
                cv2.putText(frame, f'{fps:4.1f} fps  {process_ms:4.0f} ms', (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
                draw_robot(frame, snap, per_px)
                cv2.imshow('detect', frame)
                key = cv2.waitKey(1) & 255
                if key == ord('q'):
                    break
    finally:
        try:
            send([], 'stopped')
        finally:
            if not args.video and 'reader' in locals():
                reader.stop()
            cap.release()
            sock.close()
            if not args.headless:
                cv2.destroyAllWindows()


if __name__ == '__main__':
    main()