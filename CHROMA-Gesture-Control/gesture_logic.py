"""Camera-independent gesture state machine and UDP v3 transport."""
from dataclasses import dataclass
import json
import math
import socket
import time
import uuid


@dataclass(frozen=True)
class Hand:
    captured_at: float
    pose: str
    x: float = .5
    y: float = .5
    features: tuple | None = None
    score: float | None = None
    candidate: str = ''


class GestureControl:
    """Only fresh, continuous observations can enable or sustain motion."""
    def __init__(self, speed=.25, timeout=.2):
        if not 0 < speed <= 1 or not 0 < timeout <= .3:
            raise ValueError('Invalid speed or input timeout')
        self.speed = speed
        self.timeout = timeout
        self.mode = 'DRIVE'
        self.enabled = False
        self.ready = False
        self.label = 'PAUSED: press G'
        self._pose = None
        self._since = 0.
        self._fired = False
        self._last_capture = None

    def pause(self):
        self.enabled = self.ready = False
        self._pose = None
        self._last_capture = None
        self.label = 'PAUSED: press G'

    def start(self):
        self.pause()
        self.enabled = True

    def update(self, hand, now):
        # Return wheel commands plus at most one explicit arm command.
        zero = (0., 0., None)
        if not self.enabled:
            return zero
        if (hand is None or not all(math.isfinite(v) for v in
                                   (hand.captured_at, hand.x, hand.y))
                or not 0 <= now - hand.captured_at <= self.timeout
                or not 0 <= hand.x <= 1 or not 0 <= hand.y <= 1):
            self.ready = False
            self._pose = None
            self._last_capture = None
            self.label = 'NO FRESH HAND: wheels stopped'
            return zero
        previous = self._last_capture
        if previous is not None and hand.captured_at < previous:
            self.ready = False
            self._pose = None
            self.label = 'OUT OF ORDER: wheels stopped'
            return zero
        new_frame = previous is None or hand.captured_at > previous
        if previous is not None and hand.captured_at - previous > self.timeout:
            self.ready = False
            self._pose = None
        self._last_capture = hand.captured_at
        if hand.pose != self._pose:
            self._pose, self._since, self._fired = hand.pose, hand.captured_at, False
        held = hand.captured_at - self._since
        if hand.pose == 'V':
            self.ready = False
            self.label = 'Hold V for 0.8s to change mode'
            if new_frame and held >= .8 and not self._fired:
                self.mode = 'ARM' if self.mode == 'DRIVE' else 'DRIVE'
                self._fired = True
                self.label = 'Mode changed; show centered open palm'
            return zero
        centered = abs(hand.x - .5) <= .12 and abs(hand.y - .5) <= .12
        if not self.ready:
            self.label = 'Center OPEN palm for 0.5s to enable'
            if hand.pose == 'OPEN' and centered:
                if new_frame and held >= .5:
                    self.ready = True
                    self.label = 'READY'
            else:
                self._since = hand.captured_at
            return zero
        if self.mode == 'DRIVE':
            if hand.pose != 'OPEN':
                self.ready = False
                self.label = 'STOP; center OPEN palm to resume'
                return zero
            dx, dy = hand.x - .5, .5 - hand.y
            if max(abs(dx), abs(dy)) <= .12:
                self.label = 'STOP: center'
                return zero
            if abs(dy) >= abs(dx):
                v = self.speed if dy > 0 else -self.speed
                self.label = 'FORWARD' if dy > 0 else 'BACK'
                return v, v, None
            v = self.speed * .6
            self.label = 'RIGHT' if dx > 0 else 'LEFT'
            return (v, -v, None) if dx > 0 else (-v, v, None)
        events = {'ONE': ('grip', 'open'), 'THREE': ('grip', 'close')}
        self.label = 'ARM: 1=open 3=close; OPEN to rearm'
        # Arm gestures require an open palm between every command.
        if hand.pose in events and new_frame and held >= .4 and not self._fired:
            self._fired = True
            self.ready = False
            cmd, pos = events[hand.pose]
            self.label = f'{cmd.upper()} {pos.upper()}'
            return 0., 0., (cmd, pos)
        return zero


def classify_landmarks(points):
    """Conservative geometric rules; unknown shapes produce no command.

    Points are aspect-corrected x/y coordinates. This is not a trained gesture
    classifier; keep palm facing camera and validate all poses on screen first.
    """
    if len(points) != 21 or any(not math.isfinite(c) for p in points for c in p):
        return 'UNKNOWN'
    def dist(a, b):
        return math.dist(points[a], points[b])
    def straight(a, b, c):
        u = (points[a][0]-points[b][0], points[a][1]-points[b][1])
        v = (points[c][0]-points[b][0], points[c][1]-points[b][1])
        denom = math.hypot(*u) * math.hypot(*v)
        return denom > 1e-8 and (u[0]*v[0]+u[1]*v[1])/denom < -.75
    fingers = [straight(m, m+1, m+3) and dist(m+3, 0) > 1.15*dist(m+1, 0)
               for m in (5, 9, 13, 17)]
    if all(fingers):
        return 'OPEN'
    if fingers == [True, True, False, False]:
        return 'V'
    if fingers == [True, False, False, False]:
        return 'ONE'
    if fingers == [True, True, True, False]:
        return 'THREE'
    if not any(fingers):
        palm = dist(5, 17)
        dx, dy = points[4][0]-points[2][0], points[4][1]-points[2][1]
        if (straight(2, 3, 4) and dist(4, 5) > .8*palm
                and abs(dy) > max(abs(dx)*1.5, palm*.6)):
            return 'THUMB_UP' if dy < 0 else 'THUMB_DOWN'
        return 'FIST'
    return 'UNKNOWN'


class RobotLink:
    def __init__(self, ip, port=4211):
        self.addr = (socket.gethostbyname(ip), port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))
        self.sock.setblocking(False)
        self.session = uuid.uuid4().hex[:12]
        self.seq = 0
        self.status = None
        self.status_at = 0.

    def send(self, cmd, **fields):
        self.seq += 1
        packet = {'v': 3, 's': self.session, 'q': self.seq, 'c': cmd, **fields}
        self.sock.sendto(json.dumps(packet, allow_nan=False).encode(), self.addr)

    def poll(self):
        for _ in range(32):
            try:
                data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, ConnectionResetError):
                break
            if addr != self.addr:
                continue
            try:
                status = json.loads(data)
            except (ValueError, UnicodeDecodeError):
                continue
            if isinstance(status, dict) and status.get('state') in ('IDLE', 'RUNNING', 'DONE', 'ESTOP'):
                self.status, self.status_at = status, time.monotonic()

    def running(self, now):
        return bool(self.status and self.status['state'] == 'RUNNING'
                    and 0 <= now-self.status_at < .6)

    def close(self):
        self.sock.close()