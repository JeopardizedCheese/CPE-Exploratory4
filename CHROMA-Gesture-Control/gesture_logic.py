"""Camera-independent two-hand gesture state machine and UDP v3 transport.

Roles are assigned by screen half of the mirrored image, not by MediaPipe
handedness: the hand on the screen-right half is the DRIVE hand, the hand on
the screen-left half is the COMMAND hand (swap=True flips this).
"""
from dataclasses import dataclass
import json
import math
import socket
import time
import uuid

# Firmware maps any non-zero |command| linearly onto MIN_DUTY..MAX_DUTY.
FIRMWARE_MIN_DUTY, FIRMWARE_MAX_DUTY = .71, 1.
DEFAULT_GEARS = (.15, .35, .6)
TURN_RATIO = .6
BOX = .1               # half-size of the neutral box around the drive centre
DRIVE_READY_S = .5     # centred OPEN palm arms the drive hand
REARM_S = .3           # command hand OPEN between commands
# Command hand: pose -> (event, dwell seconds, needs OPEN rearm first).
COMMANDS = {
    'ONE': (('grip', 'open'), .4, True),
    'THREE': (('grip', 'close'), .4, True),      # rules / trained MLP
    'ILOVEYOU': (('grip', 'close'), .4, True),   # MediaPipe canned model has no THREE
    'THUMB_UP': (('gear', 'up'), .4, True),
    'THUMB_DOWN': (('gear', 'down'), .4, True),
    'V': (('start', None), .8, True),
    'FIST': (('stop', None), .5, False),   # stopping never waits for a rearm
}

# MediaPipe Gesture Recognizer canned categories -> our pose names.
CANNED = {'None': 'UNKNOWN', 'Open_Palm': 'OPEN', 'Closed_Fist': 'FIST', 'Victory': 'V',
          'Pointing_Up': 'ONE', 'Thumb_Up': 'THUMB_UP', 'Thumb_Down': 'THUMB_DOWN',
          'ILoveYou': 'ILOVEYOU'}


def canned_pose(category, score, min_score):
    """Low-confidence or unrecognised canned gestures become UNKNOWN."""
    return CANNED.get(category, 'UNKNOWN') if score >= min_score else 'UNKNOWN'


def duty(command):
    """Approximate firmware PWM duty for a normalized wheel command."""
    mag = min(abs(command), 1.)
    return 0. if mag < .01 else FIRMWARE_MIN_DUTY + mag*(FIRMWARE_MAX_DUTY-FIRMWARE_MIN_DUTY)


@dataclass(frozen=True)
class Hand:
    captured_at: float
    pose: str
    x: float = .5
    y: float = .5
    features: tuple | None = None
    score: float | None = None
    candidate: str = ''


class _Track:
    """Pose dwell for one hand; stale, reordered or gapped frames reset it."""
    def __init__(self, timeout):
        self.timeout = timeout
        self.reset()

    def reset(self):
        self.pose = None
        self.since = 0.
        self.fired = False
        self.last_capture = None

    def observe(self, hand, now):
        """Return (usable, new_frame, held_seconds); usable=False means lost."""
        if (hand is None or not all(math.isfinite(v) for v in
                                   (hand.captured_at, hand.x, hand.y))
                or not 0 <= now - hand.captured_at <= self.timeout
                or not 0 <= hand.x <= 1 or not 0 <= hand.y <= 1):
            self.reset()
            return False, False, 0.
        previous = self.last_capture
        if previous is not None and (hand.captured_at < previous
                                     or hand.captured_at - previous > self.timeout):
            self.reset()
            return False, False, 0.
        new_frame = previous is None or hand.captured_at > previous
        self.last_capture = hand.captured_at
        if hand.pose != self.pose:
            self.pose, self.since, self.fired = hand.pose, hand.captured_at, False
        return True, new_frame, hand.captured_at - self.since


class GestureControl:
    """Only fresh, continuous observations can enable or sustain motion."""
    def __init__(self, gears=DEFAULT_GEARS, timeout=.2, swap=False):
        if (not gears or any(not 0 < g <= 1 for g in gears) or list(gears) != sorted(gears)
                or not 0 < timeout <= .3):
            raise ValueError('Invalid gears or input timeout')
        self.gears = tuple(gears)
        self.gear = 0
        self.timeout = timeout
        self.swap = swap
        self.enabled = False
        self.ready = False          # drive hand armed
        self.armed = False          # command hand showed OPEN since last command
        self.drive_label = self.command_label = 'PAUSED: press G or hold left V'
        self._drive = _Track(timeout)
        self._command = _Track(timeout)

    @property
    def speed(self):
        return self.gears[self.gear]

    def pause(self):
        # The command track is kept so a still-held V/FIST does not fire again.
        self.enabled = self.ready = self.armed = False
        self._drive.reset()
        self.drive_label = self.command_label = 'PAUSED: press G or hold left V'

    def start(self):
        self.pause()
        self.enabled = True

    def assign(self, hands):
        """Split hands by screen half -> (drive, command, ok)."""
        right = [h for h in hands if h.x >= .5]
        left = [h for h in hands if h.x < .5]
        if len(right) > 1 or len(left) > 1:
            return None, None, False
        drive, command = (right or [None])[0], (left or [None])[0]
        return (command, drive, True) if self.swap else (drive, command, True)

    def update(self, hands, now):
        """Return (left_wheel, right_wheel, events) for this tick."""
        drive, command, ok = self.assign(hands)
        if not ok:
            self.ready = self.armed = False
            self._drive.reset()
            self._command.reset()
            self.drive_label = self.command_label = 'TWO HANDS ON ONE SIDE: stopped'
            return 0., 0., []
        events = self._update_command(command, now)
        left, right = self._update_drive(drive, now)
        return left, right, events

    def _update_command(self, hand, now):
        usable, new_frame, held = self._command.observe(hand, now)
        if not usable:
            self.armed = False
            self.command_label = 'Command hand: none'
            return []
        pose = hand.pose
        if pose == 'OPEN':
            if new_frame and held >= REARM_S:
                self.armed = True
            self.command_label = 'Command: READY' if self.armed else 'Command: hold OPEN'
            return []
        if pose not in COMMANDS:
            self.command_label = 'Command: -'
            return []
        (event, arg), dwell, needs_rearm = COMMANDS[pose]
        if not self.enabled and event not in ('start', 'stop'):
            self.command_label = 'PAUSED: only V start / FIST stop'
            return []
        if needs_rearm and not self.armed:
            self.command_label = 'Command: show OPEN first'
            return []
        self.command_label = f'Command: {pose} {held:.1f}/{dwell:.1f}s'
        if not new_frame or held < dwell or self._command.fired:
            return []
        self._command.fired = True
        if needs_rearm:
            self.armed = False
        if event == 'gear':
            step = 1 if arg == 'up' else -1
            self.gear = min(max(self.gear+step, 0), len(self.gears)-1)
            arg = self.gear+1
        self.command_label = f'Command: {event.upper()} {arg if arg is not None else ""}'.rstrip()
        return [(event, arg)]

    def _update_drive(self, hand, now):
        if not self.enabled:
            return 0., 0.
        usable, new_frame, held = self._drive.observe(hand, now)
        if not usable:
            self.ready = False
            self.drive_label = 'NO FRESH DRIVE HAND: wheels stopped'
            return 0., 0.
        cx = .25 if self.swap else .75
        centered = abs(hand.x - cx) <= BOX and abs(hand.y - .5) <= BOX
        if not self.ready:
            self.drive_label = 'Center OPEN palm in box for 0.5s'
            if hand.pose == 'OPEN' and centered:
                if new_frame and held >= DRIVE_READY_S:
                    self.ready = True
                    self.drive_label = 'READY'
            else:
                self._drive.since = hand.captured_at
            return 0., 0.
        if hand.pose != 'OPEN':
            self.ready = False
            self.drive_label = 'STOP; center OPEN palm to resume'
            return 0., 0.
        if centered:
            self.drive_label = 'STOP: center'
            return 0., 0.
        dx, dy = hand.x - cx, .5 - hand.y
        if abs(dy) >= abs(dx):
            v = self.speed if dy > 0 else -self.speed
            self.drive_label = 'FORWARD' if dy > 0 else 'BACK'
            return v, v
        v = self.speed * TURN_RATIO
        self.drive_label = 'RIGHT' if dx > 0 else 'LEFT'
        return (v, -v) if dx > 0 else (-v, v)


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
            # robot_ctrl firmware has only IDLE and RUNNING.
            if isinstance(status, dict) and status.get('state') in ('IDLE', 'RUNNING'):
                self.status, self.status_at = status, time.monotonic()

    def running(self, now):
        return bool(self.status and self.status['state'] == 'RUNNING'
                    and 0 <= now-self.status_at < .6)

    def close(self):
        self.sock.close()
