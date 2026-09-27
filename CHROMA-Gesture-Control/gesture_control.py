"""Webcam gestures for CHROMA UDP v3. Default: preview only, no network."""
import argparse
from pathlib import Path
import threading
import time

from gesture_logic import GestureControl, Hand, RobotLink, classify_landmarks


class CameraWorker(threading.Thread):
    """Own camera/inference in a worker so a blocked read cannot repeat motion."""
    def __init__(self, camera, model, classifier=None):
        super().__init__(daemon=True)
        self.camera, self.model = camera, model
        self.classifier = classifier
        self.lock = threading.Lock()
        self.end = threading.Event()
        self.latest = (None, None, 'Starting camera...')

    def snapshot(self):
        with self.lock:
            return self.latest

    def publish(self, frame, hand, error=''):
        with self.lock:
            self.latest = frame, hand, error

    def run(self):
        cap = None
        try:
            import cv2
            import mediapipe as mp
            from gesture_model import landmark_features
            cap = cv2.VideoCapture(self.camera)
            if not cap.isOpened():
                raise RuntimeError(f'Cannot open camera {self.camera}')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            options = mp.tasks.vision.HandLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(self.model)),
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_hands=2, min_hand_detection_confidence=.7,
                min_hand_presence_confidence=.7, min_tracking_confidence=.7)
            with mp.tasks.vision.HandLandmarker.create_from_options(options) as detector:
                last_ms = -1
                while not self.end.is_set():
                    captured = time.monotonic()  # includes camera read + inference delay
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('Camera read failed')
                    frame = cv2.flip(frame, 1)  # mirror: hand right = screen right
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    timestamp = max(last_ms+1, int(captured*1000))
                    last_ms = timestamp
                    result = detector.detect_for_video(
                        mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp)
                    hand = None
                    if len(result.hand_landmarks) == 1:
                        marks = result.hand_landmarks[0]
                        h, w = frame.shape[:2]
                        features = landmark_features([(p.x, p.y, p.z) for p in marks], w, h)
                        if features is not None:
                            score, candidate = None, ''
                            if self.classifier is not None:
                                pose, score, candidate = self.classifier.predict(features)
                            else:
                                pose = classify_landmarks([(p.x*w/h, p.y) for p in marks])
                            palm = [marks[i] for i in (0, 5, 9, 13, 17)]
                            hand = Hand(captured, pose, sum(p.x for p in palm)/5,
                                        sum(p.y for p in palm)/5, tuple(features), score, candidate)
                        for p in marks:
                            cv2.circle(frame, (int(p.x*w), int(p.y*h)), 3, (0, 240, 0), -1)
                    self.publish(frame, hand)
        except Exception as exc:
            self.publish(None, None, str(exc))
        finally:
            if cap is not None:
                cap.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0, help='Operator webcam, not overhead camera')
    parser.add_argument('--model', type=Path, default=Path(__file__).parent/'models/hand_landmarker.task')
    parser.add_argument('--classifier', type=Path, help='Your trained MLP .npz; omitted = original rules')
    parser.add_argument('--robot', help='Robot IP; additionally requires --live')
    parser.add_argument('--port', type=int, default=4211)
    parser.add_argument('--live', action='store_true', help='Enable UDP transmission (otherwise preview only)')
    parser.add_argument('--speed', type=float, default=.25, help='Normalized motor command, not measured speed')
    args = parser.parse_args()
    if args.live and not args.robot:
        parser.error('--live requires --robot IP')
    if not 0 < args.speed <= .5:
        parser.error('--speed must be greater than 0 and at most 0.5 for this prototype')
    if not args.model.is_file():
        parser.error('Model missing. Run setup_gesture.ps1 first, or use --model PATH.')
    import cv2
    import numpy as np
    classifier = None
    if args.classifier:
        from gesture_model import GestureModel
        try:
            classifier = GestureModel(args.classifier)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.error(f'Cannot load classifier (no fallback to rules): {exc}')
    control = GestureControl(args.speed)
    link = RobotLink(args.robot, args.port) if args.live else None
    worker = CameraWorker(args.camera, args.model, classifier)
    last_send = 0.
    pending_start = None
    event_text = ''
    print('LIVE UDP v3' if link else 'PREVIEW ONLY: no robot packets sent')
    worker.start()
    try:
        while True:
            now = time.monotonic()
            frame, hand, error = worker.snapshot()
            view = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)
            if link:
                link.poll()
                # poll stamps received status with monotonic time. Evaluate
                # freshness AFTER polling, never against an earlier timestamp.
                now = time.monotonic()
                if pending_start is not None:
                    if link.running(now):
                        control.start()
                        pending_start = None
                    elif now-pending_start > 1.:
                        pending_start = None
                        event_text = 'START not confirmed. Check robot; press G to retry.'
                if control.enabled and not link.running(now):
                    control.pause()
                    event_text = 'Robot status lost/not RUNNING. Press G after checking.'
            left, right, event = control.update(hand, now)
            if event:
                cmd, pos = event
                event_text = f'{cmd.upper()} {pos.upper()} (requested, not physical feedback)'
                if link:
                    link.send('drive', l=0., r=0.)
                    link.send(cmd, p=pos)
            if now-last_send >= .05:
                last_send = now
                if link:
                    link.send('drive', l=left, r=right)
            h, w = view.shape[:2]
            cv2.rectangle(view, (int(w*.38), int(h*.38)), (int(w*.62), int(h*.62)), (255, 200, 0), 2)
            score_text = (f'{hand.candidate} score={hand.score:.2f}'
                          if hand is not None and hand.score is not None else '')
            lines = [f'{"LIVE" if link else "PREVIEW"} | {"MLP" if classifier else "RULES"} | {control.mode} | {"ENABLED" if control.enabled else "PAUSED"}',
                     f'Pose: {hand.pose if hand else "NONE / MULTIPLE HANDS"} | L {left:+.2f} R {right:+.2f}',
                     control.label, error or event_text,
                     'G start/resume | SPACE pause | X ESTOP | R reset | ESC quit',
                     'Hold V: change mode | Center OPEN: enable | Use ONE hand', score_text]
            for i, text in enumerate(lines):
                cv2.putText(view, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 0, 0), 3)
                cv2.putText(view, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1)
            cv2.imshow('CHROMA gesture control', view)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty('CHROMA gesture control', cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord(' '), ord('x'), ord('r')):
                control.pause()
                pending_start = None
                if link:
                    link.send('drive', l=0., r=0.)
                    if key == ord('x'):
                        link.send('stop')
                    elif key == ord('r'):
                        link.send('reset')
            elif key == ord('g'):
                control.pause()
                if link:
                    link.send('start')
                    pending_start = time.monotonic()
                else:
                    control.start()
    finally:
        control.pause()
        worker.end.set()
        if link:
            for _ in range(3):
                try:
                    link.send('drive', l=0., r=0.)
                    link.send('stop')
                except OSError:
                    pass
            link.close()
        cv2.destroyAllWindows()
        worker.join(timeout=1.)


if __name__ == '__main__':
    main()
