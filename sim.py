"""Simulated robot + field for testing the planner without hardware.

It copies firmware/robot_ctrl's behaviour (states, 300 ms drive watchdog, 5-minute
timer, rate-limited servos, status packet) and adds what the firmware can't know:
wheel kinematics, the gripper picking up stones, zones, and a simplified camera.

Simplifications, on purpose:
  - Perception is geometric, not image-based: stones are circles, 'pickable' uses
    the same idea as vision.py (clear all round, or a gripper-wide free strip).
  - The robot does not push stones. Driving through the pile is not punished, so
    watch paths in the --show window.
  - Pose noise, dropped tag frames, latency and grip failures are configurable.
"""
import math
import random
from dataclasses import dataclass

from robot_pose import Pose, footprint_polygon_mm

STONE_R = 20.0        # stones as ~40 mm circles


@dataclass
class Stone:
    x: float
    y: float
    color: int
    state: str = 'floor'      # floor | held | placed
    zone: int = 0             # colour of the zone it was dropped in (placed only)


DEFAULT_PARAMS = {
    'max_speed_mm_s': 300.0,     # speed at drive command 1.0
    'wheel_base_mm': 130.0,
    'axle_offset_mm': 0.0,       # tag centre is this far ahead of the wheel axle
    'ramp_per_s': 3.0,
    'servo_deg_per_s': 180.0,
    'grip_servo': 0, 'grip_open': 0, 'grip_close': 40,
    'servo_start': [90],
    'grip_reach_mm': 25.0,       # stone centre must be within this of the grip point (forward)
    'grip_side_mm': 18.0,        #   ... and this sideways
    'grip_success': 1.0,         # probability a well-placed grab holds
    'pose_noise_mm': 0.0, 'heading_noise_deg': 0.0,
    'tag_dropout': 0.0,          # fraction of frames without a pose
    'latency_s': 0.0,            # how old perception is when the planner gets it
    'run_time_s': None, 'drive_timeout_s': 0.3,   # firmware has no run timer; set seconds to emulate one
    # Motor/field effects, all off by default (see FIELD_PARAMS):
    'min_duty': 0.0,             # firmware MIN_DUTY: command c drives at min_duty + |c| * (1 - min_duty)
    'stall_duty': 0.0,           # below this duty the wheel does not turn; speed grows from here to 1.0
    'spin_speed_scale': 1.0,     # turning in place is slower than wheel kinematics (tyre scrub)
    'spin_breakaway': None,      # (lo, hi): duty needed to start turning in place, drawn at each start
    'wall_mm': 0.0,              # the body stops the tag this far from a wall
    'tag_edge_mm': 0.0,          # no pose within this distance of the arena edge (tag out of frame)
}

# Fitted to the 2026-09-29 field traces (runs/autonomy/20260929-02*): forward 0.3 -> ~190 mm/s,
# 0.6 -> ~370 mm/s; spin 0.3 stalls about half the time and turns ~30 deg/s on average;
# tag lost 100-170 mm from the edge. A model, not a measurement of every robot.
FIELD_PARAMS = {'min_duty': 0.71, 'stall_duty': 0.70, 'max_speed_mm_s': 600.0,
                'spin_speed_scale': 0.35, 'spin_breakaway': (0.76, 0.84),
                'wall_mm': 100.0, 'tag_edge_mm': 130.0}


def in_polygon(x, y, poly):
    inside = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def zone_color(key):
    return int(str(key).split('_')[0])


class SimRobot:
    def __init__(self, cfg, stones, x, y, heading_deg, params=None, seed=0):
        self.p = dict(DEFAULT_PARAMS, **(params or {}))
        self.rng = random.Random(seed)
        tag = cfg.get('robot_tag', {})
        self.offset = tag.get('grip_offset_mm', [120, 0])
        self.footprint = tag.get('footprint_mm', {'front': 170, 'back': 110, 'left': 105, 'right': 105})
        self.size = cfg['arena']['size_mm']
        self.zones = {zone_color(k): (z['center_mm'][0], z['center_mm'][1], z['radius_mm'])
                      for k, z in cfg.get('zones', {}).items()}
        self.vision = cfg.get('vision', {})
        self.stones = [Stone(*s) if not isinstance(s, Stone) else s for s in stones]
        self.x, self.y, self.h = float(x), float(y), math.radians(heading_deg)
        self.state, self.why = 'IDLE', 'boot'
        self.cmd = [0.0, 0.0]
        self.out = [0.0, 0.0]
        self.last_drive = -1e9
        self.run_start = 0.0
        self.servo = [float(v) for v in self.p['servo_start']]
        self.target = list(self.servo)
        self.held = None
        self.t = 0.0
        self.history = []            # (t, x, y, h) for latency
        self.spinning = False        # turning in place has broken free of static friction
        self.breakaway = self._draw_breakaway()
        self.log = []

    # ------------------------------------------------------------ firmware side
    def command(self, c, now, **f):
        if c == 'drive':
            l, r = f.get('l'), f.get('r')
            if self.state != 'RUNNING' or l is None or r is None or abs(l) > 1 or abs(r) > 1:
                self.cmd = [0.0, 0.0]
                return
            self.cmd, self.last_drive = [float(l), float(r)], now
        elif c == 'start' and self.state == 'IDLE':
            self.state, self.why, self.run_start = 'RUNNING', 'start', now
        elif c == 'stop':
            self._enter('IDLE', 'remote stop')
        elif c in ('grip', 'servo') and self.state in ('IDLE', 'RUNNING'):
            p = self.p
            if c == 'grip':
                deg = {'open': p['grip_open'], 'close': p['grip_close']}.get(f.get('p'))
                i = p['grip_servo']
            else:
                i, deg = f.get('i', -1), f.get('deg')
            if deg is not None and 0 <= i < len(self.servo):
                self.target[i] = float(max(0, min(180, deg)))

    def _enter(self, state, why):
        self.state, self.why = state, why
        if state != 'RUNNING':
            self.cmd, self.out = [0.0, 0.0], [0.0, 0.0]

    def status(self, now):
        return {'state': self.state, 'why': self.why, 'l': self.out[0],
                'r': self.out[1], 'servo': [round(v) for v in self.servo]}

    # ------------------------------------------------------------ physics
    def _draw_breakaway(self):
        b = self.p['spin_breakaway']
        return self.rng.uniform(*b) if b else 0.0

    def _wheel_speeds(self):
        """mm/s per wheel from firmware output, through MIN_DUTY, motor stall and spin friction."""
        p = self.p
        duty = [0.0 if abs(o) < 0.01 else math.copysign(p['min_duty'] + abs(o) * (1 - p['min_duty']), o)
                for o in self.out]
        spin = duty[0] * duty[1] < 0
        if not spin:
            if self.spinning:
                self.breakaway = self._draw_breakaway()
            self.spinning = False
        elif not self.spinning:
            self.spinning = min(abs(d) for d in duty) >= self.breakaway
            if not self.spinning:
                return [0.0, 0.0], spin                      # stuck: wheels hum, robot does not turn
        stall = p['stall_duty']
        speed = [math.copysign(max(0.0, abs(d) - stall) / (1 - stall), d) * p['max_speed_mm_s'] for d in duty]
        if spin:
            speed = [v * p['spin_speed_scale'] for v in speed]
        return speed, spin

    def grip_point(self, x=None, y=None, h=None):
        x = self.x if x is None else x
        y = self.y if y is None else y
        h = self.h if h is None else h
        f, rt = self.offset
        return (x + f * math.cos(h) - rt * math.sin(h), y + f * math.sin(h) + rt * math.cos(h))

    def update(self, now):
        dt = max(0.0, now - self.t)
        self.t = now
        if self.state == 'RUNNING' and self.p['run_time_s'] and now - self.run_start >= self.p['run_time_s']:
            self._enter('IDLE', 'time up')
        cmd = self.cmd if (self.state == 'RUNNING' and now - self.last_drive <= self.p['drive_timeout_s']) else [0, 0]
        for i in (0, 1):                                   # firmware ramp: up slowly, down instantly
            o, c = self.out[i], cmd[i]
            if abs(c) < abs(o) or c * o < 0:
                self.out[i] = 0.0 if c * o < 0 else c
            else:
                step = self.p['ramp_per_s'] * dt
                self.out[i] = min(c, o + step) if c > o else max(c, o - step)
        (sl, sr), _ = self._wheel_speeds()
        v = (sl + sr) / 2
        w = (sl - sr) / self.p['wheel_base_mm']            # rad/s, clockwise
        a = self.p['axle_offset_mm']
        ax, ay = self.x - a * math.cos(self.h), self.y - a * math.sin(self.h)     # rotate about the axle
        self.h += w * dt
        ax += v * math.cos(self.h) * dt
        ay += v * math.sin(self.h) * dt
        self.x, self.y = ax + a * math.cos(self.h), ay + a * math.sin(self.h)
        m = self.p['wall_mm']
        self.x = min(max(self.x, m), self.size[0] - m)
        self.y = min(max(self.y, m), self.size[1] - m)
        for i in range(len(self.servo)):
            d = self.target[i] - self.servo[i]
            s = self.p['servo_deg_per_s'] * dt
            self.servo[i] += max(-s, min(s, d))
        self._gripper()
        self.history.append((now, self.x, self.y, self.h))
        self.history = [e for e in self.history if now - e[0] <= self.p['latency_s'] + 0.5]

    def _gripper(self):
        p = self.p
        grip = self.servo[p['grip_servo']]
        gx, gy = self.grip_point()
        if self.held is None and abs(grip - p['grip_close']) < 2:
            f = (math.cos(self.h), math.sin(self.h))
            best = None
            for s in self.stones:
                if s.state != 'floor':
                    continue
                dx, dy = s.x - gx, s.y - gy
                along, side = dx * f[0] + dy * f[1], -dx * f[1] + dy * f[0]
                if abs(along) <= p['grip_reach_mm'] and abs(side) <= p['grip_side_mm']:
                    d = math.hypot(dx, dy)
                    if best is None or d < best[0]:
                        best = (d, s)
            if best and self.rng.random() < p['grip_success']:
                self.held = best[1]
                self.held.state = 'held'
                self.log.append(('grabbed', self.held.color))
        if self.held is not None:
            self.held.x, self.held.y = gx, gy
            if grip < p['grip_close'] - 15:                # opened: drop it here
                s, self.held = self.held, None
                s.state = 'floor'
                for c, (zx, zy, zr) in self.zones.items():
                    if math.hypot(s.x - zx, s.y - zy) <= zr:
                        s.state, s.zone = 'placed', c
                self.log.append(('dropped', s.color, s.zone))

    # ------------------------------------------------------------ camera side
    def score(self):
        placed = [s for s in self.stones if s.state == 'placed']
        return {'correct': sum(s.zone == s.color for s in placed),
                'wrong': sum(s.zone != s.color for s in placed)}

    def perceive(self, now):
        """(pose, targets, observations) as the real Perception would report them."""
        t_seen = now - self.p['latency_s']
        past = min(self.history, key=lambda e: abs(e[0] - t_seen)) if self.history else (now, self.x, self.y, self.h)
        _, x, y, h = past
        pose = None
        edge = min(x, y, self.size[0] - x, self.size[1] - y)
        if self.rng.random() >= self.p['tag_dropout'] and edge >= self.p['tag_edge_mm']:
            x += self.rng.gauss(0, self.p['pose_noise_mm'])
            y += self.rng.gauss(0, self.p['pose_noise_mm'])
            h += math.radians(self.rng.gauss(0, self.p['heading_noise_deg']))
            gx, gy = self.grip_point(x, y, h)
            pose = Pose(x, y, math.degrees(math.atan2(math.sin(h), math.cos(h))), gx, gy, 100.0, t_seen)
        body = [tuple(pt) for pt in footprint_polygon_mm(
            Pose(self.x, self.y, math.degrees(self.h), 0, 0, 0, now), self.footprint)]
        visible = [s for s in self.stones if s.state == 'floor' and not in_polygon(s.x, s.y, body)
                   and not any(math.hypot(s.x - zx, s.y - zy) <= zr + 20 for zx, zy, zr in self.zones.values())]
        observations = [{'color': s.color, 'x': s.x, 'y': s.y} for s in visible]
        targets = []
        for s in visible:
            approach = self._approach(s, visible, body)
            if approach is not False:
                t = {'color': s.color, 'x': round(s.x, 1), 'y': round(s.y, 1), 'confidence': 0.8}
                if approach is not None:
                    t['approach_deg'] = approach
                targets.append(t)
        return pose, targets, observations

    def _blocked(self, px, py, own, visible, body):
        if px < 20 or py < 20 or px > self.size[0] - 20 or py > self.size[1] - 20:
            return True
        if in_polygon(px, py, body):
            return True
        if any(math.hypot(px - zx, py - zy) <= zr + 20 for zx, zy, zr in self.zones.values()):
            return True
        return any(o is not own and math.hypot(px - o.x, py - o.y) <= STONE_R for o in visible)

    def _approach(self, s, visible, body):
        """None = clear all round, heading in degrees = pile approach, False = not pickable."""
        clearance = self.vision.get('clearance_mm', 60)
        others = [o for o in visible if o is not s]
        near = min((math.hypot(o.x - s.x, o.y - s.y) for o in others), default=1e9)
        if near - 2 * STONE_R > clearance and not self._blocked(s.x, s.y, s, visible, body):
            return None
        if not self.vision.get('pile_mode', False):
            return False
        width = self.vision.get('gripper_width_mm', 60)
        length = self.vision.get('approach_length_mm', 80)
        reach = length + width + STONE_R
        visible = [o for o in visible if abs(o.x - s.x) < reach and abs(o.y - s.y) < reach]
        nearest = min(others, key=lambda o: math.hypot(o.x - s.x, o.y - s.y), default=None)
        preferred = math.atan2(s.y - nearest.y, s.x - nearest.x) if nearest else 0.0
        for k in sorted(range(16), key=lambda k: abs(math.remainder(k * math.pi / 8, 2 * math.pi))):
            d = preferred + k * math.pi / 8 if k <= 8 else preferred - (16 - k) * math.pi / 8
            if abs(math.remainder(d - preferred, 2 * math.pi)) > math.radians(67.5) + 1e-6:
                continue
            ux, uy = math.cos(d), math.sin(d)
            free = True
            for a in range(int(STONE_R), int(length) + 1, 10):
                for b in (-width / 2, 0, width / 2):
                    if self._blocked(s.x + ux * a - uy * b, s.y + uy * a + ux * b, s, visible, body):
                        free = False
                        break
                if not free:
                    break
            if free:
                return round(math.degrees(math.atan2(-uy, -ux)), 1)
        return False


def pile(cx, cy, colors, spacing=44.0, seed=0):
    """Stones packed in a roughly round pile (touching-ish), like the match start."""
    rng = random.Random(seed)
    spots = []
    ring = 0
    while len(spots) < len(colors):
        if ring == 0:
            spots.append((cx, cy))
        else:
            n = 6 * ring
            for k in range(n):
                a = 2 * math.pi * k / n
                spots.append((cx + ring * spacing * math.cos(a) + rng.uniform(-3, 3),
                              cy + ring * spacing * math.sin(a) + rng.uniform(-3, 3)))
        ring += 1
    colors = list(colors)
    rng.shuffle(colors)
    return [Stone(x, y, c) for (x, y), c in zip(spots, colors)]
