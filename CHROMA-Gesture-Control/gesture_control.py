"""Two-hand webcam gestures for robot_ctrl UDP v3. Default: preview only, no network."""
import argparse
from pathlib import Path
import threading
import time

from gesture_logic import (BOX, DEFAULT_GEARS, GestureControl, Hand, RobotLink,
                           canned_pose, classify_landmarks, duty)


class CameraWorker(threading.Thread):
    """Own camera/inference in a worker so a blocked read cannot repeat motion."""
    def __init__(self, camera, model, classifier=None, recognizer=None, min_score=.7):
        """model: hand landmarker; recognizer: MediaPipe gesture_recognizer.task (replaces it)."""
        super().__init__(daemon=True)
        self.camera, self.model = camera, model
        self.classifier = classifier
        self.recognizer, self.min_score = recognizer, min_score
        self.lock = threading.Lock()
        self.end = threading.Event()
        self.latest = (None, (), 'Starting camera...')

    def snapshot(self):
        with self.lock:
            return self.latest

    def publish(self, frame, hands, error=''):
        with self.lock:
            self.latest = frame, hands, error

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
            vision = mp.tasks.vision
            common = dict(running_mode=vision.RunningMode.VIDEO, num_hands=2,
                          min_hand_detection_confidence=.7, min_hand_presence_confidence=.7,
                          min_tracking_confidence=.7)
            if self.recognizer is not None:
                detector = vision.GestureRecognizer.create_from_options(vision.GestureRecognizerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=str(self.recognizer)), **common))
                detect = detector.recognize_for_video
            else:
                detector = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
                    base_options=mp.tasks.BaseOptions(model_asset_path=str(self.model)), **common))
                detect = detector.detect_for_video
            with detector:
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
                    result = detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp)
                    hands = []
                    h, w = frame.shape[:2]
                    for i, marks in enumerate(result.hand_landmarks):
                        features = landmark_features([(p.x, p.y, p.z) for p in marks], w, h)
                        if features is not None:
                            score, candidate = None, ''
                            if self.recognizer is not None:
                                top = result.gestures[i][0] if result.gestures[i] else None
                                candidate = top.category_name if top else 'None'
                                score = top.score if top else 0.
                                pose = canned_pose(candidate, score, self.min_score)
                            elif self.classifier is not None:
                                pose, score, candidate = self.classifier.predict(features)
                            else:
                                pose = classify_landmarks([(p.x*w/h, p.y) for p in marks])
                            palm = [marks[i] for i in (0, 5, 9, 13, 17)]
                            hands.append(Hand(captured, pose, sum(p.x for p in palm)/5,
                                              sum(p.y for p in palm)/5, tuple(features), score, candidate))
                        for p in marks:
                            cv2.circle(frame, (int(p.x*w), int(p.y*h)), 3, (0, 240, 0), -1)
                    self.publish(frame, tuple(hands))
        except Exception as exc:
            self.publish(None, (), str(exc))
        finally:
            if cap is not None:
                cap.release()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', type=int, default=0, help='Operator webcam, not overhead camera')
    parser.add_argument('--model', type=Path, default=Path(__file__).parent/'models/hand_landmarker.task')
    parser.add_argument('--gesture-model', type=Path,
                        default=Path(__file__).parent/'models/gesture_recognizer.task',
                        help='MediaPipe pretrained Gesture Recognizer (default pose source)')
    parser.add_argument('--min-score', type=float, default=.7,
                        help='MediaPipe gesture score below this -> UNKNOWN')
    parser.add_argument('--classifier', type=Path, help='Use your trained MLP .npz instead of MediaPipe gestures')
    parser.add_argument('--rules', action='store_true', help='Use the original geometric rules instead')
    parser.add_argument('--robot', default='10.178.188.50', help='Robot IP (firmware static IP); used only with --live')
    parser.add_argument('--port', type=int, default=4211)
    parser.add_argument('--live', action='store_true', help='Enable UDP transmission (otherwise preview only)')
    parser.add_argument('--gears', default=','.join(map(str, DEFAULT_GEARS)),
                        help='Wheel command per gear, ascending, 0..1. Firmware maps 0<|cmd|<=1 onto duty 0.71..1.0')
    parser.add_argument('--swap-hands', action='store_true', help='Drive with the LEFT hand, command with the RIGHT')
    args = parser.parse_args()
    try:
        gears = tuple(float(g) for g in args.gears.split(','))
        control = GestureControl(gears, swap=args.swap_hands)
    except ValueError:
        parser.error('--gears must be ascending numbers in (0, 1], e.g. 0.15,0.35,0.6')
    if args.rules and args.classifier:
        parser.error('Choose one of --rules or --classifier')
    use_mediapipe = not (args.rules or args.classifier)
    if not 0 < args.min_score <= 1:
        parser.error('--min-score must be in (0, 1]')
    if not args.model.is_file():
        parser.error('Model missing. Run setup_gesture.ps1 first, or use --model PATH.')
    if use_mediapipe and not args.gesture_model.is_file():
        parser.error('Gesture model missing. Run setup_gesture.ps1, or use --rules / --classifier.')
    import cv2
    import numpy as np
    classifier = None
    if args.classifier:
        from gesture_model import GestureModel
        try:
            classifier = GestureModel(args.classifier)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            parser.error(f'Cannot load classifier (no fallback to rules): {exc}')
    link = RobotLink(args.robot, args.port) if args.live else None
    worker = CameraWorker(args.camera, args.model, classifier,
                          args.gesture_model if use_mediapipe else None, args.min_score)
    source = 'MEDIAPIPE' if use_mediapipe else 'MLP' if classifier else 'RULES'
    last_send = 0.
    pending_start = None
    event_text = ''
    print(f'LIVE UDP v3 -> {args.robot}:{args.port}' if link else 'PREVIEW ONLY: no robot packets sent')

    def request_start():
        nonlocal pending_start
        control.pause()
        if link:
            link.send('start')
            pending_start = time.monotonic()
        else:
            control.start()

    def request_stop():
        # robot_ctrl: stop -> IDLE (wheels off). Not latched; G / left V starts again.
        nonlocal pending_start
        control.pause()
        pending_start = None
        if link:
            link.send('drive', l=0., r=0.)
            link.send('stop')

    worker.start()
    try:
        while True:
            now = time.monotonic()
            frame, hands, error = worker.snapshot()
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
            left, right, events = control.update(hands, now)
            for cmd, arg in events:
                if cmd == 'grip':
                    event_text = f'GRIP {arg.upper()} (requested, see servo angle)'
                    if link:
                        link.send('grip', p=arg)
                elif cmd == 'gear':
                    event_text = f'GEAR {arg}'
                elif cmd == 'start':
                    event_text = 'START requested'
                    request_start()
                elif cmd == 'stop':
                    event_text = 'STOP -> IDLE'
                    request_stop()
                    left = right = 0.
            if now-last_send >= .05:
                last_send = now
                if link:
                    link.send('drive', l=left, r=right)
            h, w = view.shape[:2]
            cx = .25 if args.swap_hands else .75
            cv2.line(view, (w//2, 0), (w//2, h), (160, 160, 160), 1)
            cv2.rectangle(view, (int(w*(cx-BOX)), int(h*(.5-BOX))),
                          (int(w*(cx+BOX)), int(h*(.5+BOX))), (255, 200, 0), 2)
            status = link.status if link else None
            robot = 'no status'
            if status:
                stale = '' if now-link.status_at < .6 else ' (STALE)'
                robot = f'{status["state"]} servo={(status.get("servo") or ["?"])[0]}{stale}'
            poses = ' '.join(f'{"R" if hand.x >= .5 else "L"}:{hand.pose}'
                             + (f'({hand.score:.2f})' if hand.score is not None else '') for hand in hands)
            lines = [f'{"LIVE" if link else "PREVIEW"} | {source} | '
                     f'{"ENABLED" if control.enabled else "PAUSED"} | robot {robot if link else "-"}',
                     f'GEAR {control.gear+1}/{len(control.gears)} cmd {control.speed:.2f} '
                     f'~duty {duty(control.speed):.2f} | L {left:+.2f} R {right:+.2f}',
                     f'Hands: {poses or "NONE"}',
                     f'Drive ({"left" if args.swap_hands else "right"} half): {control.drive_label}',
                     control.command_label, error or event_text,
                     'G start | SPACE pause | X stop(IDLE) | ESC quit',
                     'Cmd hand: point-up grip open | ILoveYou/3 grip close | thumb up/down gear | V start | FIST stop']
            for i, text in enumerate(lines):
                cv2.putText(view, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, .46, (0, 0, 0), 3)
                cv2.putText(view, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, .46, (255, 255, 255), 1)
            cv2.imshow('CHROMA gesture control', view)
            key = cv2.waitKey(10) & 255
            if key == 27 or cv2.getWindowProperty('CHROMA gesture control', cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' '):
                control.pause()
                pending_start = None
                if link:
                    link.send('drive', l=0., r=0.)
            elif key == ord('x'):
                request_stop()
            elif key == ord('g'):
                request_start()
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
