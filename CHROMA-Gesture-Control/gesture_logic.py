"""Two hands, one command set: either hand can drive, so the driver can swap hands.

Each visible hand is classified on its own into a command (gesture_model.LABELS).
The hands are combined into ONE command per frame:
  no fresh hand -> NO_HAND (stop)        any hand STOP -> STOP
  two different commands -> CONFLICT (stop)   only NONE -> NONE (stop)
  otherwise the one command shown (the other hand may rest, or show the same).
Motion needs the command held MOVE_HOLD_S; anything else stops the wheels at once.
A grip command fires once after GRIP_HOLD_S; change the command to fire again.
All decisions are free of camera/network I/O.
"""
from dataclasses import dataclass
import json
import math
import socket
import time
import uuid

# Firmware maps any non-zero |command| linearly onto MIN_DUTY..MAX_DUTY.
FIRMWARE_MIN_DUTY, FIRMWARE_MAX_DUTY = .71, 1.
TURN_RATIO = .6                      # turning in place, as teleop.py a/d
SPEED_MIN, SPEED_STEP = .1, .1       # +/- keys, as teleop.py
MOVE_HOLD_S = .15                    # a motion command must be steady this long
GRIP_HOLD_S = .4
MOTION = {'FORWARD': (1., 1.), 'BACK': (-1., -1.),
          'LEFT': (-TURN_RATIO, TURN_RATIO), 'RIGHT': (TURN_RATIO, -TURN_RATIO)}
GRIP = {'GRIP_OPEN': 'open', 'GRIP_CLOSE': 'close'}
COMMANDS = ('STOP', *MOTION, *GRIP)
STOPPED = {'NO_HAND': 'No hand seen: stopped', 'NONE': 'No command: stopped',
           'CONFLICT': 'Hands disagree: stopped', 'STOP': 'STOP'}


def duty(command, floor=FIRMWARE_MIN_DUTY):
    """Approximate firmware PWM duty for a normalized wheel command and floor duty."""
    mag = min(abs(command), 1.)
    return 0. if mag < .01 else floor + mag*(FIRMWARE_MAX_DUTY-floor)


@dataclass(frozen=True)
class Hand:
    captured_at: float
    pose: str                       # command label (or NONE)
    x: float = .5                   # palm centre, mirrored image, 0..1
    y: float = .5
    features: tuple | None = None
    score: float | None = None
    candidate: str = ''             # arg-max label before the confidence gate
    side: str = ''                  # 'L' / 'R' as MediaPipe sees it ('' = unknown)
    points: tuple = ()              # 21 (x, y) landmarks, 0..1, for drawing


def fresh(hand, now, timeout):
    return (hand is not None
            and all(math.isfinite(v) for v in (hand.captured_at, hand.x, hand.y))
            and 0 <= now - hand.captured_at <= timeout
            and 0 <= hand.x <= 1 and 0 <= hand.y <= 1)


def combine(hands, now, timeout):
    """-> (command, indexes of the hands giving it)."""
    usable = [i for i, h in enumerate(hands) if fresh(h, now, timeout)]
    if not usable:
        return 'NO_HAND', ()
    said = {i: hands[i].pose for i in usable if hands[i].pose in COMMANDS}
    if 'STOP' in said.values():
        return 'STOP', tuple(i for i, c in said.items() if c == 'STOP')
    if len(set(said.values())) > 1:
        return 'CONFLICT', tuple(said)
    if not said:
        return 'NONE', ()
    return next(iter(said.values())), tuple(said)


class GestureDrive:
    """Only fresh, steady commands move the wheels; everything else stops them."""
    def __init__(self, speed=.3, timeout=.2):
        if not math.isfinite(speed) or not SPEED_MIN <= speed <= 1:
            raise ValueError(f'speed must be in [{SPEED_MIN}, 1]')
        if not 0 < timeout <= .3:
            raise ValueError('timeout must be in (0, 0.3]')
        self.speed = round(speed, 2)
        self.timeout = timeout
        self.enabled = False
        self.command, self.active = 'NO_HAND', ()
        self.since = self.frame_t = None
        self.fired = False
        self.progress = 0.          # 0..1 of the hold time, for the UI
        self.message = 'Paused: press G to start'

    def adjust_speed(self, steps):
        """+/- keys: change the speed by SPEED_STEP, at once."""
        self.speed = round(min(1., max(SPEED_MIN, self.speed + steps*SPEED_STEP)), 2)
        return self.speed

    def _restart_hold(self):
        self.since = self.frame_t = None
        self.fired = False
        self.progress = 0.

    def start(self):
        self.enabled = True
        self._restart_hold()      # a pose held while paused must be held again
        self.message = 'Running: show a command'

    def pause(self):
        self.enabled = False
        self._restart_hold()
        self.message = 'Paused: press G to start'

    def update(self, hands, now):
        """Return (left_wheel, right_wheel, events) for this tick."""
        command, active = combine(hands, now, self.timeout)
        t = max((hands[i].captured_at for i in active), default=None)
        if t is None and command == 'NONE':
            t = max(h.captured_at for h in hands if fresh(h, now, self.timeout))
        if command != self.command or t is None or (self.frame_t is not None and t < self.frame_t):
            self._restart_hold()
            self.since = t
        elif self.since is None:        # start() cleared the hold: it begins at this frame
            self.since = t
        self.command, self.active, self.frame_t = command, active, t
        held = 0. if t is None or self.since is None else t - self.since
        zero = (0., 0., [])
        if not self.enabled:
            self.progress = 0.
            self.message = 'Paused: press G to start'
            return zero
        if command in MOTION:
            self.progress = min(1., held / MOVE_HOLD_S)
            if held < MOVE_HOLD_S:
                self.message = f'{command}: hold...'
                return zero
            self.message = command
            left, right = MOTION[command]
            return left*self.speed, right*self.speed, []
        if command in GRIP:
            self.progress = 1. if self.fired else min(1., held / GRIP_HOLD_S)
            name = command.replace('_', ' ')
            if self.fired:
                self.message = f'{name} sent; change pose to send again'
                return zero
            if held < GRIP_HOLD_S:
                self.message = f'{name}: hold...'
                return zero
            self.fired = True
            self.message = f'{name} sent'
            return 0., 0., [('grip', GRIP[command])]
        self.progress = 0.
        self.message = STOPPED[command]
        return zero


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
