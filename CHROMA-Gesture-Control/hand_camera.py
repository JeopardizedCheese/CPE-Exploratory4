"""Operator webcam + MediaPipe hand landmarks in a worker thread (a blocked read cannot repeat motion)."""
import threading
import time

from gesture_logic import Hand


def operator_side(category_name):
    """MediaPipe handedness of our mirrored frame -> the operator's real hand ('L' / 'R').

    On this frame MediaPipe's label comes out reversed (raising the left hand read "Right",
    seen on the team's webcam 2026-10-01), so it is swapped here.
    """
    return {'Left': 'R', 'Right': 'L'}.get(category_name, '')


class CameraWorker(threading.Thread):
    """Publishes (mirrored frame, hands, error). With a classifier each hand gets a command label."""
    def __init__(self, camera, model, classifier=None):
        super().__init__(daemon=True)
        self.camera, self.model, self.classifier = camera, model, classifier
        self.lock = threading.Lock()
        self.end = threading.Event()
        self.latest = (None, (), 'Starting camera...')
        self.fps = 0.

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
            detector = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
                base_options=mp.tasks.BaseOptions(model_asset_path=str(self.model)),
                running_mode=vision.RunningMode.VIDEO, num_hands=2,
                min_hand_detection_confidence=.7, min_hand_presence_confidence=.7,
                min_tracking_confidence=.7))
            with detector:
                last_ms, last_t = -1, None
                while not self.end.is_set():
                    captured = time.monotonic()  # includes camera read + inference delay
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('Camera read failed')
                    frame = cv2.flip(frame, 1)  # mirror: hand right = screen right
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    timestamp = max(last_ms+1, int(captured*1000))
                    last_ms = timestamp
                    result = detector.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb),
                                                       timestamp)
                    hands = []
                    h, w = frame.shape[:2]
                    for i, marks in enumerate(result.hand_landmarks):
                        features = landmark_features([(p.x, p.y, p.z) for p in marks], w, h)
                        if features is None:
                            continue
                        pose, score, candidate = 'NONE', None, ''
                        if self.classifier is not None:
                            pose, score, candidate = self.classifier.predict(features)
                        side = ''
                        if i < len(result.handedness) and result.handedness[i]:
                            side = operator_side(result.handedness[i][0].category_name)
                        palm = [marks[k] for k in (0, 5, 9, 13, 17)]
                        hands.append(Hand(captured, pose, sum(p.x for p in palm)/5, sum(p.y for p in palm)/5,
                                          tuple(features), score, candidate, side,
                                          tuple((p.x, p.y) for p in marks)))
                    if last_t is not None and captured > last_t:
                        self.fps = .9*self.fps + .1/(captured-last_t) if self.fps else 1/(captured-last_t)
                    last_t = captured
                    self.publish(frame, tuple(hands))
        except Exception as exc:
            self.publish(None, (), str(exc))
        finally:
            if cap is not None:
                cap.release()
