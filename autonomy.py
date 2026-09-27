"""Autonomous run: pick one stone, carry it to its colour's zone, repeat.

    python autonomy.py --sim --show                 # simulated robot and field, watch it
    python autonomy.py --sim --scenario scattered   # easier field
    python autonomy.py 192.168.1.50 --camera 1      # real robot (sends 'start', 5-minute run)

The planner is deliberately simple:

  SEARCH -> GOTO_STAGE -> ALIGN -> APPROACH -> GRIP -> CARRY -> RELEASE -> BACKOFF
     ^                                                                    |
     +--------------------------------------------------------------------+

  SEARCH     lock the cheapest target (robot -> stone -> zone), open the gripper
  GOTO_STAGE drive to a point stage_mm behind the stone, on its approach line
  ALIGN      turn in place to the approach heading
  APPROACH   creep in until the grip point reaches the stone (the robot now hides it)
  GRIP       close; waits until the ESP32 reports the grip servo arrived
  CARRY      slide the stone along the floor to the centre of its zone (colour fixed at lock)
  RELEASE/BACKOFF  open, reverse, repeat

The gripper has no lift: stones stay on the floor and are pushed/slid in the closed jaws.

Failures never drop a stone in the wrong place: approach/align/goto time out into a
short backoff and the stone is skipped for a while; a missed grab is noticed when the
stone is still visible at its old spot after the robot has left, and the gripper is
only opened over the correct zone.

All tuning values live in calib.json -> "autonomy" (defaults below). Servo angles
there must equal firmware/robot_ctrl/config.h.
"""
import argparse
import json
import math
import threading
import time
from pathlib import Path

from target_lock import TargetLock

DEFAULTS = {
    'cruise': 0.45, 'creep': 0.18, 'turn': 0.35, 'min_turn': 0.16,
    'kp_turn': 1.0,                 # drive-command per radian of heading error
    'turn_in_place_deg': 35,        # larger heading error: stop and turn first
    'stage_mm': 160, 'stage_tol_mm': 35, 'align_tol_deg': 6,
    'grip_tol_mm': 6, 'approach_max_side_mm': 18,   # about half of (open jaw gap - stone width)
    'zone_tol_mm': 45, 'backoff_s': 0.9,
    'pose_timeout_s': 0.5, 'servo_tol_deg': 3, 'servo_timeout_s': 2.0,
    'timeouts_s': {'GOTO_STAGE': 15, 'ALIGN': 6, 'APPROACH': 8, 'SEARCH_IDLE': 2.5, 'PARK': 10},
    'skip_s': 25, 'skip_mm': 40, 'pick_check_mm': 180, 'pile_avoid_mm': 170,
    'grip_open': 0, 'grip_close': 40, 'grip_servo': 0,   # = GRIP_OPEN/CLOSE_DEG in config.h
    'park_mm': None,                 # where to wait when nothing is pickable; default right side
    'stone_height_mm': 20,
}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Planner:
    def __init__(self, cfg):
        self.o = dict(DEFAULTS, **cfg.get('autonomy', {}))
        self.o['timeouts_s'] = dict(DEFAULTS['timeouts_s'], **cfg.get('autonomy', {}).get('timeouts_s', {}))
        tag = cfg.get('robot_tag', {})
        self.offset = tag.get('grip_offset_mm', [120, 0])
        self.axle = float(tag.get('axle_offset_mm', 0))    # tag centre is this far ahead of the wheel axle
        self.footprint = tag.get('footprint_mm', {'front': 170, 'back': 110, 'left': 105, 'right': 105})
        self.zones = {int(str(k).split('_')[0]): (z['center_mm'][0], z['center_mm'][1], z['radius_mm'])
                      for k, z in cfg.get('zones', {}).items()}
        w, h = cfg['arena']['size_mm']
        self.park = self.o['park_mm'] or [w * 0.85, h * 0.5]
        self.lock = TargetLock(max_missing_s=1.0, match_mm=35)
        self.state, self.since = 'SEARCH', 0.0
        self.skip = []                      # (x, y, until)
        self.pick_pos = None
        self.pick_checked = False
        self.uncovered_at = None            # when the pick spot came back into view
        self.side_avg = None                # smoothed sideways error during APPROACH
        self.discard_to = None
        self.retries = 0
        self.heading = 0.0                  # approach heading (rad)
        self.carrying = None
        self.placed = 0
        self.events_log = []
        self.pile_center = None

    # ------------------------------------------------------------ helpers
    def _go(self, s, now, why=''):
        if s != self.state:
            self.events_log.append((round(now, 2), self.state, s, why))
        self.state, self.since = s, now

    def _elapsed(self, now):
        return now - self.since

    def _servo_at(self, status, which, deg):
        if not status or 'servo' not in status:
            return True                     # no status: assume it arrived after the wait
        i = self.o[f'{which}_servo']
        return abs(status['servo'][i] - deg) <= self.o['servo_tol_deg']

    def _servo_done(self, now, status, which, deg):
        return self._servo_at(status, which, deg) or self._elapsed(now) > self.o['servo_timeout_s']

    def _skip_target(self, now, why):
        t = self.lock.target
        if t:
            self.skip.append((t['x'], t['y'], now + self.o['skip_s']))
        self.lock.release(why)

    def _usable(self, t, now):
        if t['color'] not in self.zones:
            return False
        return not any(now < until and math.hypot(t['x'] - x, t['y'] - y) < self.o['skip_mm']
                       for x, y, until in self.skip)

    def _approach_heading(self, t, pose):
        if t.get('approach_deg') is not None:
            return math.radians(t['approach_deg'])
        return math.atan2(t['y'] - pose.y, t['x'] - pose.x)

    def _centre_for(self, t, heading, extra):
        """Wheel-axle position that puts the grip point `extra` mm before the stone.
        The axle is what stays put when the robot turns on the spot, so after ALIGN the
        gripper is still on the approach line."""
        f, r = self.offset
        f += self.axle
        fx, fy = math.cos(heading), math.sin(heading)
        return (t['x'] - fx * (f + extra) + fy * r, t['y'] - fy * (f + extra) - fx * r)

    def _axle(self, pose):
        h = math.radians(pose.heading_deg)
        return pose.x - self.axle * math.cos(h), pose.y - self.axle * math.sin(h)

    def _under_robot(self, pose, x, y, margin=40):
        h = math.radians(pose.heading_deg)
        dx, dy = x - pose.x, y - pose.y
        f, r = dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)
        fp = self.footprint
        return -fp['back'] - margin <= f <= fp['front'] + margin and -fp['left'] - margin <= r <= fp['right'] + margin

    def _in_any_zone(self, x, y, margin=60):
        return any(math.hypot(x - zx, y - zy) <= zr + margin for zx, zy, zr in self.zones.values())

    def _safe_drop(self, pose):
        """Nearest point toward the park spot that is well outside every zone."""
        x, y = pose.grip_x, pose.grip_y
        for k in range(0, 21):
            px = x + (self.park[0] - x) * k / 20
            py = y + (self.park[1] - y) * k / 20
            if not self._in_any_zone(px, py, 90):
                return px, py
        return tuple(self.park)

    def _crosses_pile(self, ax, ay, bx, by):
        if self.pile_center is None:
            return False
        px, py = self.pile_center
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1.0
        k = clamp(((px - ax) * dx + (py - ay) * dy) / L2, 0, 1)
        return math.hypot(ax + k * dx - px, ay + k * dy - py) < self.o['pile_avoid_mm']

    def _choose(self, pose):
        def cost(t):
            a = self._approach_heading(t, pose)
            sx, sy = self._centre_for(t, a, self.o['stage_mm'])
            zx, zy, _ = self.zones[t['color']]
            ax, ay = self._axle(pose)
            c = math.hypot(sx - ax, sy - ay) + math.hypot(zx - t['x'], zy - t['y'])
            c += 600 * self._crosses_pile(ax, ay, sx, sy)
            return c - 100 * t.get('confidence', 0)
        return lambda targets: min(targets, key=cost) if targets else None

    def _arrived(self, pose, px, py, gx, gy, tol):
        """Close enough, or so close that the goal swings behind us (turning would oscillate)."""
        dist = math.hypot(gx - px, gy - py)
        err = wrap(math.atan2(gy - py, gx - px) - math.radians(pose.heading_deg))
        return dist < tol or (dist < 1.5 * tol and abs(err) > math.radians(self.o['turn_in_place_deg']))

    def _drive_to(self, pose, px, py, gx, gy, speed):
        """Steer point (px, py) of the robot (centre or grip point) towards (gx, gy)."""
        dist = math.hypot(gx - px, gy - py)
        err = wrap(math.atan2(gy - py, gx - px) - math.radians(pose.heading_deg))
        o = self.o
        if abs(err) > math.radians(o['turn_in_place_deg']):
            w = math.copysign(max(o['min_turn'], min(o['turn'], o['kp_turn'] * abs(err))), err)
            return w, -w
        v = speed * clamp(dist / 200.0, 0.4, 1.0) * math.cos(err)
        w = clamp(o['kp_turn'] * err, -o['turn'], o['turn'])
        return clamp(v + w, -1, 1), clamp(v - w, -1, 1)

    def _turn_to(self, pose, heading):
        err = wrap(heading - math.radians(pose.heading_deg))
        if abs(err) <= math.radians(self.o['align_tol_deg']):
            return None
        o = self.o
        w = math.copysign(max(o['min_turn'], min(o['turn'], o['kp_turn'] * abs(err))), err)
        return w, -w

    # ------------------------------------------------------------ main step
    def step(self, now, pose, targets, observations, status=None):
        """Returns (l, r, events). events: [(cmd, fields)] one-off commands to send."""
        ev = []
        if status and status.get('state') not in (None, 'RUNNING'):
            return 0.0, 0.0, ev
        if pose is None or now - pose.t > self.o['pose_timeout_s']:
            return 0.0, 0.0, ev                          # no fresh pose: stand still
        if observations:
            xs = [o['x'] for o in observations]
            ys = [o['y'] for o in observations]
            self.pile_center = (sorted(xs)[len(xs) // 2], sorted(ys)[len(ys) // 2])
        o, s = self.o, self.state
        usable = [t for t in targets if self._usable(t, now)]
        heading = math.radians(pose.heading_deg)

        if s == 'SEARCH':
            t = self.lock.update(usable, now, observations, choose=self._choose(pose))
            if t:
                last = getattr(self, '_last_locked', None)          # same stone again = a retry
                if last is None or math.hypot(t['x'] - last[0], t['y'] - last[1]) > 40:
                    self.retries = 0
                self._last_locked = (t['x'], t['y'])
                self.heading = self._approach_heading(t, pose)
                ev.append(('grip', {'p': 'open'}))
                self._go('GOTO_STAGE', now, f"colour {t['color']}")
            elif self._elapsed(now) > o['timeouts_s']['SEARCH_IDLE']:
                self._go('PARK', now, 'nothing pickable')
            return 0.0, 0.0, ev

        if s == 'PARK':
            if usable or self._elapsed(now) > o['timeouts_s']['PARK']:
                self._go('SEARCH', now)
                return 0.0, 0.0, ev
            if math.hypot(self.park[0] - pose.x, self.park[1] - pose.y) < 60:
                return 0.0, 0.0, ev
            return (*self._drive_to(pose, pose.x, pose.y, self.park[0], self.park[1], o['cruise']), ev)

        if s == 'GOTO_STAGE':
            t = self.lock.update(usable, now, observations)
            if t is None:
                self._go('SEARCH', now, f'target {self.lock.reason}')
                return 0.0, 0.0, ev
            sx, sy = self._centre_for(t, self.heading, o['stage_mm'])   # heading frozen at lock
            ux, uy = math.cos(self.heading), math.sin(self.heading)
            gx, gy = t['x'] - pose.grip_x, t['y'] - pose.grip_y
            along, side = gx * ux + gy * uy, -gx * uy + gy * ux
            on_line = (40 < along < o['stage_mm'] + 60 and abs(side) < 12
                       and abs(wrap(self.heading - heading)) < math.radians(25))
            ax, ay = self._axle(pose)
            if on_line or self._arrived(pose, ax, ay, sx, sy, o['stage_tol_mm']):
                self._go('ALIGN', now)
                return 0.0, 0.0, ev
            if self._elapsed(now) > o['timeouts_s']['GOTO_STAGE']:
                self._skip_target(now, 'stage timeout')
                self._go('BACKOFF', now, 'stage timeout')
                return 0.0, 0.0, ev
            return (*self._drive_to(pose, ax, ay, sx, sy, o['cruise']), ev)

        if s == 'ALIGN':
            t = self.lock.update(usable, now, observations, occluded=True)
            if t is None:
                self._go('SEARCH', now, f'target {self.lock.reason}')
                return 0.0, 0.0, ev
            cmd = self._turn_to(pose, self.heading)
            ready = self._servo_at(status, 'grip', o['grip_open'])
            if cmd is None and ready:
                self.side_avg = None
                self._go('APPROACH', now)
                return 0.0, 0.0, ev
            if self._elapsed(now) > o['timeouts_s']['ALIGN']:
                self._skip_target(now, 'align timeout')
                self._go('BACKOFF', now, 'align timeout')
                return 0.0, 0.0, ev
            return (*(cmd or (0.0, 0.0)), ev)

        if s == 'APPROACH':
            t = self.lock.update(usable, now, observations, occluded=True)
            if t is None:
                self._go('SEARCH', now, f'target {self.lock.reason}')
                return 0.0, 0.0, ev
            # Follow the approach line through the stone: along = distance still to go,
            # side = how far the grip point is off that line (not relative to our heading,
            # which swings the grip point whenever we steer).
            ux, uy = math.cos(self.heading), math.sin(self.heading)
            dx, dy = t['x'] - pose.grip_x, t['y'] - pose.grip_y
            along, side = dx * ux + dy * uy, -dx * uy + dy * ux
            self.side_avg = side if self.side_avg is None else 0.6 * self.side_avg + 0.4 * side
            side = self.side_avg
            if along <= o['grip_tol_mm']:
                if abs(side) > o['approach_max_side_mm']:
                    if self.retries < 1:                 # back up and line up once more
                        self.retries += 1
                    else:
                        self._skip_target(now, 'missed sideways')
                    self._go('BACKOFF', now, f'side error {side:.0f} mm')
                    return 0.0, 0.0, ev
                self.pick_pos, self.pick_checked, self.uncovered_at = (t['x'], t['y']), False, None
                self.carrying = t['color']
                ev.append(('grip', {'p': 'close'}))
                self._go('GRIP', now)
                return 0.0, 0.0, ev
            if self._elapsed(now) > o['timeouts_s']['APPROACH']:
                self._skip_target(now, 'approach timeout')
                self._go('BACKOFF', now, 'approach timeout')
                return 0.0, 0.0, ev
            wanted = self.heading + math.atan2(side, o.get('approach_lookahead_mm', 150))
            err = wrap(wanted - heading)
            if abs(err) > math.radians(20):                  # badly off: turn on the spot first
                w = math.copysign(o['min_turn'], err)
                return w, -w, ev
            steer = clamp(0.8 * err, -0.12, 0.12)
            v = o['creep']
            return v + steer, v - steer, ev

        if s == 'GRIP':
            if self._servo_done(now, status, 'grip', o['grip_close']):
                self._go('CARRY', now, f'to zone {self.carrying}')
            return 0.0, 0.0, ev

        if s == 'CARRY':
            zx, zy, _ = self.zones[self.carrying]
            if not self.pick_checked and self.pick_pos:
                px, py = self.pick_pos
                if self.uncovered_at is None and not self._under_robot(pose, px, py):
                    self.uncovered_at = now             # vision needs a few frames to confirm
                if self.uncovered_at is not None and now - self.uncovered_at > o.get('pick_check_s', 0.7):
                    self.pick_checked = True
                    if any(ob['color'] == self.carrying and math.hypot(ob['x'] - px, ob['y'] - py) < 35
                           for ob in observations):
                        # The stone is still there: we hold nothing, or a neighbour of unknown
                        # colour. Never open inside a zone: drop it somewhere neutral.
                        self.lock.release('grab missed')
                        self.carrying = None
                        self.discard_to = self._safe_drop(pose)
                        self._go('DISCARD', now, 'grab missed')
                        return 0.0, 0.0, ev
            zr = self.zones[self.carrying][2]
            if self._arrived(pose, pose.grip_x, pose.grip_y, zx, zy, o['zone_tol_mm']) and \
                    math.hypot(zx - pose.grip_x, zy - pose.grip_y) <= zr * 0.6:   # never open outside the zone
                ev.append(('grip', {'p': 'open'}))
                self._go('RELEASE', now)
                return 0.0, 0.0, ev
            return (*self._drive_to(pose, pose.grip_x, pose.grip_y, zx, zy, o['cruise']), ev)

        if s == 'DISCARD':
            dx, dy = self.discard_to
            if self._in_any_zone(pose.grip_x, pose.grip_y, 90) and self._elapsed(now) < 15:
                return (*self._drive_to(pose, pose.grip_x, pose.grip_y, dx, dy, o['cruise']), ev)
            ev.append(('grip', {'p': 'open'}))
            self._go('BACKOFF', now, 'discarded')
            return 0.0, 0.0, ev

        if s == 'RELEASE':
            if self._servo_done(now, status, 'grip', o['grip_open']):
                self.placed += 1
                self.carrying = None
                self.lock.done()
                self._go('BACKOFF', now, 'placed')
            return 0.0, 0.0, ev

        if s == 'BACKOFF':
            if self._elapsed(now) > o['backoff_s']:
                self._go('SEARCH', now)
                return 0.0, 0.0, ev
            return -o['creep'] * 1.5, -o['creep'] * 1.5, ev

        return 0.0, 0.0, ev


def grip_calibration_warning(cfg):
    tag = cfg.get('robot_tag', {})
    missing = []
    if not tag.get('grip_calibrated'):
        missing.append('grip offset: python calibrate_grip.py <camera> --write  (stone in closed jaws, arm down)')
    if 'axle_offset_mm' not in tag:
        missing.append('axle offset: python calibrate_grip.py <camera> --axle --write  (spin on the spot)')
    return ('WARNING, not measured yet:\n  ' + '\n  '.join(missing)) if missing else None


# ------------------------------------------------------------ simulation runner
def run_sim(cfg, stones, seconds=300.0, start=None, params=None, seed=0, show=False, rate=10.0):
    import sim
    w, h = cfg['arena']['size_mm']
    x, y, hd = start or (w * 0.92, h * 0.5, 180.0)
    robot = sim.SimRobot(cfg, stones, x, y, hd, params=params, seed=seed)
    planner = Planner(cfg)
    dt, t, next_frame = 0.02, 0.0, 0.0
    pose, targets, obs = None, [], []
    l = r = 0.0
    robot.command('start', 0.0)
    while t < seconds and robot.state == 'RUNNING':
        robot.update(t)
        if t >= next_frame:                                  # camera/planner rate
            next_frame += 1.0 / rate
            pose, targets, obs = robot.perceive(t)
            l, r, events = planner.step(t, pose, targets, obs, robot.status(t))
            for cmd, fields in events:
                robot.command(cmd, t, **fields)
            if show and not _show(robot, planner, targets, t):
                break
        robot.command('drive', t, l=l, r=r)                  # 50 Hz, like the sender thread
        t += dt
    return {**robot.score(), 'placed_by_planner': planner.placed, 'time_s': round(t, 1),
            'log': planner.events_log, 'robot': robot}


def _show(robot, planner, targets, t, scale=0.4):
    import cv2
    import numpy as np
    from robot_pose import Pose, footprint_polygon_mm
    w, h = robot.size
    img = np.full((int(h * scale), int(w * scale), 3), 235, np.uint8)
    bgr = {1: (140, 40, 120), 2: (160, 140, 0), 3: (40, 30, 200), 4: (30, 130, 240), 5: (230, 190, 120), 6: (60, 200, 110)}
    P = lambda x, y: (int(x * scale), int(y * scale))
    for c, (zx, zy, zr) in robot.zones.items():
        cv2.circle(img, P(zx, zy), int(zr * scale), bgr[c], 2)
    for s in robot.stones:
        cv2.circle(img, P(s.x, s.y), int(sim_r(scale)), bgr[s.color], -1 if s.state != 'placed' else 2)
    for tg in targets:
        cv2.circle(img, P(tg['x'], tg['y']), 11, (0, 170, 0), 1)
    body = footprint_polygon_mm(Pose(robot.x, robot.y, math.degrees(robot.h), 0, 0, 0, t), robot.footprint)
    cv2.polylines(img, [np.array([P(*p) for p in body], np.int32)], True, (60, 60, 60), 2)
    cv2.arrowedLine(img, P(robot.x, robot.y), P(*robot.grip_point()), (0, 0, 0), 2)
    if planner.lock.target:
        tg = planner.lock.target
        cv2.circle(img, P(tg['x'], tg['y']), 14, (0, 0, 255), 2)
    sc = robot.score()
    cv2.putText(img, f"{t:5.1f}s  {planner.state}  correct {sc['correct']}  wrong {sc['wrong']}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 1)
    cv2.imshow('autonomy sim', img)
    return (cv2.waitKey(1) & 255) not in (ord('q'), 27)


def sim_r(scale):
    return max(3, 20 * scale)


def scenario(cfg, name, seed=0):
    import random
    import sim
    colors = [c for c in range(1, 7) for _ in range(9)]
    w, h = cfg['arena']['size_mm']
    if name == 'pile':
        return sim.pile(w * 0.45, h * 0.5, colors, seed=seed)
    rng = random.Random(seed)
    stones = []
    while len(stones) < 18:
        x, y = rng.uniform(250, w - 450), rng.uniform(200, h - 200)
        far_zone = all(math.hypot(x - z['center_mm'][0], y - z['center_mm'][1]) > z['radius_mm'] + 120
                       for z in cfg.get('zones', {}).values())
        if far_zone and all(math.hypot(x - s.x, y - s.y) > 150 for s in stones):
            stones.append(sim.Stone(x, y, colors[len(stones) * 3 % 54]))
    return stones


# ------------------------------------------------------------ real robot runner
class DriveSender:
    """Sends the latest (l, r) 20x per second; zeros if the planner goes quiet."""

    def __init__(self, link, rate=20.0, stale_s=0.25):
        self.link, self.rate, self.stale_s = link, rate, stale_s
        self.l = self.r = 0.0
        self.t = 0.0
        self.ok = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set(self, l, r):
        with self.lock:
            self.l, self.r, self.t = l, r, time.monotonic()

    def _run(self):
        while self.ok:
            with self.lock:
                fresh = time.monotonic() - self.t <= self.stale_s
                l, r = (self.l, self.r) if fresh else (0.0, 0.0)
                self.link.send('drive', l=round(l, 3), r=round(r, 3))
            time.sleep(1.0 / self.rate)

    def stop(self):
        self.ok = False
        self.thread.join(timeout=1.0)


def run_real(args, cfg):
    import cv2
    from detect_live import LatestFrame
    from perception import Perception, draw_robot
    from teleop import Link
    warning = grip_calibration_warning(cfg)
    if warning:
        print(warning)
    background_path = args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(background_path)) if background_path.exists() else None
    if background is None:
        raise SystemExit('No background.png: run calibrate_arena.py first')
    perception = Perception(cfg, background)
    planner = Planner(cfg)
    cap = cv2.VideoCapture(args.camera if args.camera is not None else cfg.get('camera_index', 0))
    if not cap.isOpened():
        raise SystemExit('Cannot open camera')
    from camera_io import apply_camera_properties
    apply_camera_properties(cap, cfg)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    reader = LatestFrame(cap)
    link = Link(args.esp_ip, args.port)
    sender = DriveSender(link)
    per_px = float(cfg['arena'].get('mm_per_px', 2))
    link.send('start')
    try:
        while True:
            ok, raw = reader.read()
            if not ok:
                break
            now = time.monotonic()
            link.poll()
            snap = perception.step(raw, now)
            status = link.status if link.status_age() < 1.0 else None
            l, r, events = planner.step(now, snap.pose, snap.targets, snap.observations, status)
            for cmd, fields in events:
                link.send(cmd, **fields)
            sender.set(l, r)
            if not args.headless:
                frame = draw_robot(snap.frame, snap, per_px)
                if planner.lock.target:
                    t = planner.lock.target
                    cv2.circle(frame, (round(t['x'] / per_px), round(t['y'] / per_px)), 14, (0, 0, 255), 2)
                state = status['state'] if status else 'NO LINK'
                cv2.putText(frame, f'{state}  {planner.state}  placed {planner.placed}  {snap.status}',
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                if warning:
                    cv2.putText(frame, 'grip/axle offset not calibrated', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
                cv2.imshow('autonomy', frame)
                if (cv2.waitKey(1) & 255) in (ord('q'), 27, ord('x')):
                    break
            if status and status.get('state') == 'IDLE' and status.get('why') == 'remote stop':
                break
    finally:
        sender.stop()
        link.send('drive', l=0, r=0)
        link.send('stop')
        reader.stop()
        cap.release()
        cv2.destroyAllWindows()
        print('placed', planner.placed, 'events:', planner.events_log[-10:])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip', nargs='?')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--camera', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--headless', action='store_true')
    p.add_argument('--sim', action='store_true', help='simulated robot and field')
    p.add_argument('--scenario', choices=['pile', 'scattered'], default='pile')
    p.add_argument('--show', action='store_true', help='draw the simulation')
    p.add_argument('--seconds', type=float, default=300)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--noise', action='store_true', help='sim: pose noise, dropped frames, latency, failed grabs')
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    if args.sim:
        params = ({'pose_noise_mm': 4, 'heading_noise_deg': 1.5, 'tag_dropout': 0.1,
                   'latency_s': 0.12, 'grip_success': 0.85} if args.noise else None)
        result = run_sim(cfg, scenario(cfg, args.scenario, args.seed), args.seconds, params=params,
                         seed=args.seed, show=args.show)
        for entry in result['log']:
            print(entry)
        print(f"correct {result['correct']}  wrong {result['wrong']}  in {result['time_s']} s "
              f"-> score {5 * result['correct'] - result['wrong']}")
        return
    if not args.esp_ip:
        p.error('esp_ip required (or use --sim)')
    run_real(args, cfg)


if __name__ == '__main__':
    main()
