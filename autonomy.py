"""Autonomous run: pick one stone, carry it to its colour's zone, repeat.

    python autonomy.py --sim --show                 # simulated robot and field, watch it
    python autonomy.py --sim --scenario scattered   # easier field
    python autonomy.py 10.178.188.50 --camera 1     # real robot (sends 'start'; q/x/ESC stops)
    python autonomy.py --dry-run --camera 1       # camera/planner only, no robot commands
    python autonomy.py 10.178.188.50 --camera 1 --record   # also save video.avi for replay
    python autonomy.py 10.178.188.50 --camera 1 --record --set min_duty=0.5   # lower drive floor
    python autonomy2.py 10.178.188.50 --camera 1 --record   # V2: pile fix + outermost + commit

This file runs V1, the field-tested behaviour; autonomy2.py runs V2 with the same calib.json
and the same flags. The differences are switches, listed in profiles.py.

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

Approach/align/goto time out into a
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
import uuid
from pathlib import Path

from robot_pose import Pose
from target_lock import TargetLock
from wall_guard import WallGuard, DEFAULTS as WALL_DEFAULTS

DEFAULTS = {
    **WALL_DEFAULTS,
    'cruise': 0.45, 'creep': 0.18, 'turn': 0.35, 'min_turn': 0.16,
    'kp_turn': 1.0,                 # drive-command per radian of heading error (fixed-power turns)
    'max_forward_steer_ratio': 0.5, # below 1: neither wheel reverses in the forward branch; with
                                    # MIN_DUTY, 0.8 made 'forward' nearly a pivot (overshoot, hunting)
    'turn_in_place_deg': 35,        # larger heading error: stop and turn first
    'stage_mm': 160, 'stage_tol_mm': 35, 'align_tol_deg': 6,
    'grip_tol_mm': 6, 'approach_max_side_mm': 18,   # about half of (open jaw gap - stone width)
    'zone_tol_mm': 45,
    'backoff_mm': 120, 'backoff_s': 1.5,    # reverse this far after a pick/skip; time cap
    # The camera shows a command's effect ~0.2 s later (measured). Plan from where the robot
    # is now: the last pose moved on by the recent velocity for this long. At rest nothing
    # changes; at 500 mm/s it is 100 mm. false = plan from the raw (old) pose.
    'camera_delay_s': 0.2, 'predict_pose': True,
    'pose_timeout_s': 0.25, 'servo_tol_deg': 3, 'servo_timeout_s': 2.0,
    'timeouts_s': {'GOTO_STAGE': 15, 'ALIGN': 6, 'APPROACH': 8, 'SEARCH_IDLE': 2.5, 'PARK': 10},
    'skip_s': 25, 'skip_mm': 40, 'pick_check_mm': 180, 'pile_avoid_mm': 170,
    'grip_open': 0, 'grip_close': 40, 'grip_servo': 0,   # = GRIP_OPEN/CLOSE_DEG in config.h
    'park_mm': None,                 # where to wait when nothing is pickable; default right side
    'stone_height_mm': 20,
    # Turning in place is done in pulses: turn for a short time sized to part of the remaining
    # angle, stop, wait until the camera shows where the robot ended up, repeat. The camera
    # reports a turn ~0.2 s after the command, and once turning the robot does 150-300 deg/s,
    # so continuous control reacts ~60 deg late; held at full turn power it often did not turn
    # at all. 'fixed' = the old continuous fixed-power turns.
    'turn_mode': 'pulse',
    'pulse_power': 0.5,              # spin command during a pulse (the firmware ramps up to it)
    'pulse_fraction': 0.7,           # plan each pulse for this share of the remaining angle
    'pulse_max_deg': 60,             # never plan more than this in one pulse
    'pulse_min_s': 0.07, 'pulse_max_s': 0.4,
    'pulse_settle_s': 0.25,          # stopped after a pulse: command-to-camera delay + one frame
    'turn_exit_deg': 15,             # a turn toward a drive goal continues until this close
                                     # (hysteresis against turn_in_place_deg: no turn/drive dance)
    'turn_keep_dir_deg': 150,        # goal almost behind: keep the current turn direction
    'backout_inside_reach': True,    # CARRY goal inside the grip point's turning circle: creep
                                     # straight back a few cm instead of turning on the spot forever
    'kp_drive': 0.5,                 # steering per radian while driving forward
    # Floor duty of the firmware's drive mapping: any non-zero command starts at this duty and
    # scales linearly to full power. None = the firmware's own MIN_DUTY (0.71). A number is
    # sent with every drive packet (firmware that reports min_duty), so it can be changed per
    # run: --set min_duty=0.5. PULSE_TABLE was measured at 0.71: with a lower floor the first
    # pulses turn less, until pulse_gain has learned the new robot.
    'min_duty': None,
    # Practice fields with fewer zones: deliver a colour that has no zone of its own to another
    # colour's zone, e.g. {"1": 3} = violet stones go to the crimson zone. The planner treats
    # such a stone as that colour from the moment it is seen (the real colour stays in
    # 'raw_color'). Colours with no zone and no alias are never picked, only avoided.
    # Competition: leave empty (an aliased stone scores as wrong). Off per run: --set color_alias={}
    'color_alias': {},
    # V2 switches (profiles.py; V1 = these defaults). commit_target: a locked stone stays locked
    # while it is still seen at its spot, even when vision no longer offers it as a target.
    # skip_alone_s: a stone skipped after a failed attempt is retried after this many seconds
    # when it is the only stone on offer (instead of parking for the whole skip_s). None = off.
    'commit_target': False,
    'skip_alone_s': None,
}


# Median rotation (deg) of a turn pulse from rest, by pulse length (s), measured on the
# field with a charged battery (runs/autonomy/20260929-07*). Spread is about 3x either way;
# Planner.pulse_gain learns this robot's factor on the run.
PULSE_TABLE = [(0.0, 0.0), (0.07, 6.0), (0.1, 12.0), (0.15, 20.0), (0.2, 31.0), (0.3, 55.0), (0.4, 78.0)]


def pulse_degrees(seconds):
    for (t0, d0), (t1, d1) in zip(PULSE_TABLE, PULSE_TABLE[1:]):
        if seconds <= t1:
            return d0 + (d1 - d0) * max(0.0, seconds - t0) / (t1 - t0)
    return PULSE_TABLE[-1][1]


def pulse_seconds(degrees):
    for (t0, d0), (t1, d1) in zip(PULSE_TABLE, PULSE_TABLE[1:]):
        if degrees <= d1:
            return t0 + (t1 - t0) * max(0.0, degrees - d0) / (d1 - d0)
    return PULSE_TABLE[-1][0]


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
        self.alias = {int(k): int(v) for k, v in (self.o['color_alias'] or {}).items()}
        w, h = cfg['arena']['size_mm']
        self.wall = WallGuard(cfg, self.o)
        self.park = self.wall.clamp_tag_goal(*(self.o['park_mm'] or [w * 0.85, h * 0.5]))
        self._wall_resume_state = None
        self.lock = TargetLock(max_missing_s=1.0, match_mm=35, track_observations=bool(self.o['commit_target']))
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
        self.released = 0
        self.events_log = []
        self.pile_center = None
        self.motion = []                    # (t, x, y, heading_deg, l, r) for stall diagnostics
        self.backoff_from = None            # where BACKOFF started reversing
        self._pulse = None                  # current turn pulse: timing, direction, start heading
        self.pulse_gain = 1.0               # learned: this robot turns gain x PULSE_TABLE
        self._stuck_pulses = 0              # consecutive pulses that did not turn the robot
        self._pulse_done_t = -1e9           # when the last pulse's wait ended (pose time)
        self._spin_dir = 0.0
        self._was_spinning = False          # _drive_to: inside a turn toward the goal
        self.debug = {}

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

    def _relabel(self, t):
        c = self.alias.get(t['color'])
        return t if c is None else dict(t, color=c, raw_color=t['color'])

    def _usable(self, t, now, skip_for=None):
        """skip_for: count a skip entry only for this many seconds after it was made
        (skip_alone_s); None = for its full skip_s."""
        if t['color'] not in self.zones:
            return False
        return not any(now < until and (skip_for is None or now < until - self.o['skip_s'] + skip_for)
                       and math.hypot(t['x'] - x, t['y'] - y) < self.o['skip_mm']
                       for x, y, until in self.skip)

    def _jaw_stone(self, pose, observations, now):
        """A known-colour stone already between the open jaws (any observation, not only
        targets: a stone in the jaws is never 'isolated')."""
        h = math.radians(pose.heading_deg)
        for ob in observations:
            if not self._usable(ob, now):
                continue
            dx, dy = ob['x'] - pose.grip_x, ob['y'] - pose.grip_y
            along, side = dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)
            if abs(along) <= self.o['grip_tol_mm'] and abs(side) <= self.o['approach_max_side_mm']:
                return ob
        return None

    def _start_grip(self, t, now, ev, why=''):
        self.pick_pos, self.pick_checked, self.uncovered_at = (t['x'], t['y']), False, None
        self.carrying = t['color']
        ev.append(('grip', {'p': 'close'}))
        self._go('GRIP', now, why)

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

    def _carry_waypoint(self, ax, ay, bx, by):
        """Next point for the grip point on the way from a to zone centre b: b itself,
        or a point beside the nearest other zone the straight line would cross. A loose
        stone pushed into a zone scores (or costs) like a placed one."""
        clear = max(self.footprint['left'], self.footprint['right']) + 30
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1.0
        worst = None
        for c, (zx, zy, zr) in self.zones.items():
            if c == self.carrying:
                continue
            k = clamp(((zx - ax) * dx + (zy - ay) * dy) / L2, 0, 1)
            if k == 0:                              # already heading away from this zone
                continue
            px, py = ax + k * dx, ay + k * dy
            d = math.hypot(px - zx, py - zy)
            if d < zr + clear and (worst is None or k < worst[0]):
                worst = (k, px, py, zx, zy, zr, d)
        if worst is None:
            return bx, by
        _, px, py, zx, zy, zr, d = worst
        if d < 1:                                   # line goes through the centre: pass on the left
            px, py, d = zx - dy / math.sqrt(L2), zy + dx / math.sqrt(L2), 1.0
        out = (zr + clear) * 1.15
        return zx + (px - zx) / d * out, zy + (py - zy) / d * out

    def _choose(self, pose):
        def cost(t):
            a = self._approach_heading(t, pose)
            sx, sy = self._centre_for(t, a, self.o['stage_mm'])
            zx, zy, _ = self.zones[t['color']]
            ax, ay = self._axle(pose)
            c = math.hypot(sx - ax, sy - ay) + math.hypot(zx - t['x'], zy - t['y'])
            c += 600 * self._crosses_pile(ax, ay, sx, sy)
            return c - 100 * t.get('confidence', 0)
        def choose(targets):
            safe = [t for t in targets if self._approach_inside(t, self._approach_heading(t, pose))]
            return min(safe, key=cost) if safe else None
        return choose

    def _approach_inside(self, target, heading):
        """Check the tag AND body at staging and pickup, not just the stone."""
        c, s = math.cos(heading), math.sin(heading)
        for extra in (self.o['stage_mm'], 0):
            ax, ay = self._centre_for(target, heading, extra)
            x, y = ax+self.axle*c, ay+self.axle*s
            p = Pose(x, y, math.degrees(heading), target['x']-extra*c,
                     target['y']-extra*s, 0, 0)
            if not self.wall.safe_pose(p):
                return False
        return True

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
        self.debug.update(goal_x_mm=gx, goal_y_mm=gy, goal_distance_mm=dist,
                          heading_error_deg=math.degrees(err))
        turning = o['turn_mode'] == 'pulse' and self._was_spinning
        if o['backout_inside_reach'] and abs(err) > math.pi / 2:
            ax, ay = self._axle(pose)
            reach = math.hypot(px - ax, py - ay)
            if reach > 50 and math.hypot(gx - ax, gy - ay) < 0.9 * reach:
                # Turning on the spot swings (px, py) round a circle that never reaches the
                # goal (it would turn back and forth forever). Back straight up a little.
                self.debug['reason'] = 'back_out_of_reach'
                return -o['creep'], -o['creep']
        if turning and abs(err) > math.radians(o['turn_keep_dir_deg']) and err * self._spin_dir < 0:
            err = math.copysign(abs(err), self._spin_dir)   # +-180 flip: do not reverse the turn
        if abs(err) > math.radians(o['turn_in_place_deg']) or turning or self._pulse:
            cmd = self._turn_step(err, pose, o['turn_exit_deg'] if o['turn_mode'] == 'pulse'
                                  else o['turn_in_place_deg'])
            if cmd is not None:
                self.debug['reason'] = 'turn_to_goal'
                self._was_spinning = True
                return cmd
        self._was_spinning = False
        v = speed * clamp(dist / 200.0, 0.4, 1.0) * math.cos(err)
        # Near the goal, v gets smaller while the old steering term did not.
        # At 80 mm / 20 degrees it gave (+.518, -.180), another pivot instead
        # of forward travel. MIN_DUTY in firmware makes that reversal pronounced.
        steer_limit = min(o['turn'], abs(v) * clamp(o['max_forward_steer_ratio'], 0, .95))
        w = clamp(o.get('kp_drive', o['kp_turn']) * err, -steer_limit, steer_limit)
        self.debug['reason'] = 'drive_forward'
        return clamp(v + w, -1, 1), clamp(v - w, -1, 1)

    def _turn_to(self, pose, heading):
        err = wrap(heading - math.radians(pose.heading_deg))
        self.debug.update(heading_error_deg=math.degrees(err), reason='turn_to_approach')
        return self._turn_step(err, pose, self.o['align_tol_deg'])

    def _turn_step(self, err, pose, done_deg):
        """One frame of turning in place toward heading error err (rad). Returns (l, r), or
        None once the robot is at rest within done_deg (never in the middle of a pulse)."""
        o = self.o
        if o['turn_mode'] != 'pulse':
            if abs(err) <= math.radians(done_deg):
                return None
            w = math.copysign(max(o['min_turn'], min(o['turn'], o['kp_turn'] * abs(err))), err)
            return w, -w
        now, pl = pose.t, self._pulse
        if pl is not None:
            if now < pl['end']:
                self.debug.update(turn_phase='turn_pulse', turn_pulse_s=pl['end'] - pl['start'])
                w = pl['dir'] * o['pulse_power']
                return w, -w
            if now < pl['settle']:
                self.debug['turn_phase'] = 'turn_settle'
                return 0.0, 0.0
            self._finish_pulse(pose)
        if abs(err) <= math.radians(done_deg):
            return None
        wanted = min(abs(math.degrees(err)) * o['pulse_fraction'], o['pulse_max_deg'])
        length = clamp(pulse_seconds(wanted / self.pulse_gain), o['pulse_min_s'], o['pulse_max_s'])
        direction = math.copysign(1.0, err)
        self._pulse = {'start': now, 'end': now + length, 'settle': now + length + o['pulse_settle_s'],
                       'dir': direction, 'heading0': pose.heading_deg,
                       'planned_deg': self.pulse_gain * pulse_degrees(length)}
        self._spin_dir = direction
        self.debug.update(turn_phase='turn_pulse', turn_pulse_s=length, turn_planned_deg=wanted)
        w = direction * o['pulse_power']
        return w, -w

    def _finish_pulse(self, pose):
        """The camera now shows where the last pulse ended: learn how far pulses turn."""
        pl, self._pulse = self._pulse, None
        self._pulse_done_t = pose.t
        moved =pl['dir'] * math.degrees(wrap(math.radians(pose.heading_deg - pl['heading0'])))
        self._stuck_pulses = self._stuck_pulses + 1 if moved < 2 else 0
        if pl['planned_deg'] > 3:
            ratio = clamp(max(moved, 0.5) / pl['planned_deg'], 0.25, 4.0)
            self.pulse_gain = clamp(self.pulse_gain * math.sqrt(ratio), 0.3, 3.0)
        self.debug.update(turn_moved_deg=moved, pulse_gain=self.pulse_gain)

    # ------------------------------------------------------------ main step
    def step(self, now, pose, targets, observations, status=None, *,
             perception_status='ok', require_status=False):
        """Plan one frame; diagnostics explain both commands and stop conditions.

        Real runs require fresh firmware status and valid perception. Simulation
        callers may omit status; neither a lost tag nor invalid vision is bypassed.
        """
        if self.alias:
            targets = [self._relabel(t) for t in targets]
            observations = [self._relabel(o) for o in observations]
        before = self.state
        self.debug = {'state': before, 'reason': '', 'targets': len(targets),
                      'observations': len(observations), 'perception': perception_status,
                      'pose_age_s': None if pose is None else now-pose.t,
                      'firmware_state': status.get('state') if status else None}
        if perception_status != 'ok':
            self.wall.pause(now, 'wall_wait_valid_vision')
            result = (0.0, 0.0, [])
            self.debug['reason'] = 'vision_' + perception_status
        elif require_status and not status:
            self.wall.pause(now, 'wall_wait_firmware')
            result = (0.0, 0.0, [])
            self.debug['reason'] = 'no_fresh_firmware_status'
        else:
            result = self._step(now, pose, targets, observations, status)
        l, r, events = result
        pl = self._pulse
        if pl is not None and ((now < pl['end'] and l * r >= 0) or now > pl['settle'] + 0.5):
            self._pulse = None              # cut short by another state, or stale: learn nothing
        if self.state != before:
            why = self.events_log[-1][3] if self.events_log else ''
            self.debug['reason'] = f'{before}->{self.state}' + (f': {why}' if why else '')
        if not self.debug['reason']:
            self.debug['reason'] = 'waiting_' + self.state.lower()
        self.debug.update(state=self.state, l=l, r=r, lock_reason=self.lock.reason,
                          target=dict(self.lock.target) if self.lock.target else None,
                          stalled=self._stalled(now, pose, l, r))
        self.debug.update(self.wall.diagnostics(pose))
        return result

    def _recover_wall(self, now, pose, observations):
        forecast = self._predict(pose) if self.o['predict_pose'] and not self.wall.active else None
        carrying = -1 if self.state == 'DISCARD' or self._wall_resume_state == 'DISCARD' else self.carrying
        command = self.wall.step(now, pose, observations, carrying, forecast)
        in_wall_state = self.state in ('WALL_RECOVERY', 'WALL_BLOCKED')
        if command is None and not in_wall_state:
            return None
        if not in_wall_state:
            self._wall_resume_state = self.state
            self._pulse, self._was_spinning = None, False
            self.motion.clear()
            self.backoff_from = None
        self.debug['reason'] = self.wall.reason
        # completed, or a retry after WALL_BLOCKED found the robot already safe (moved by hand)
        if self.wall.completed or command is None:
            resume = self._wall_resume_state
            self._wall_resume_state = None
            self.motion.clear()
            if resume == 'CARRY' and self.carrying is not None:
                self._go('CARRY', now, 'wall recovered; keep payload')
            elif resume == 'DISCARD':
                self.discard_to = self._safe_drop(pose)
                self._go('DISCARD', now, 'wall recovered')
            else:
                self._skip_target(now, 'wall recovery')
                self._go('SEARCH', now, 'wall recovered; choose a new approach')
        else:
            self._go('WALL_BLOCKED' if self.wall.failed else 'WALL_RECOVERY', now, self.wall.reason)
        return None if command is None else (*command, [])

    def _predict(self, pose):
        """pose moved on by camera_delay_s at the velocity of the last ~0.15 s of poses."""
        if self._pulse is not None or pose.t - self._pulse_done_t < 0.25:
            return pose     # turn pulses measure at rest; the pulse's own motion is not a velocity
        # velocity over >= 0.12 s: a shorter baseline (e.g. just after a missed tag frame)
        # turns a few mm of tag noise into a fast "motion" and a 20 mm jump
        past = [m for m in self.motion if 0.12 <= pose.t - m[0] <= 0.25]
        if not past:
            return pose
        t0, x0, y0, h0 = past[0][:4]
        # Fade in between 60 and 120 mm/s: below that it is mostly tag jitter, and a hard
        # threshold would switch the prediction on and off (the pose jumping 10-20 mm).
        # Real driving is faster: creep ~400 mm/s charged, ~100 mm/s on a flat battery.
        speed = math.hypot(pose.x - x0, pose.y - y0) / (pose.t - t0)
        k = self.o['camera_delay_s'] / (pose.t - t0) * clamp((speed - 60) / 60, 0.0, 1.0)
        dx = clamp((pose.x - x0) * k, -200, 200)
        dy = clamp((pose.y - y0) * k, -200, 200)
        dh = 0.0            # pulse mode: heading changes only in pulses, measured at rest
        if self.o['turn_mode'] != 'pulse':
            dh = clamp(math.degrees(wrap(math.radians(pose.heading_deg - h0))) * k, -60, 60)
        a = math.radians(dh)
        gx, gy = pose.grip_x - pose.x, pose.grip_y - pose.y
        x, y = pose.x + dx, pose.y + dy
        self.debug.update(predict_mm=math.hypot(dx, dy), predict_deg=dh)
        return Pose(x, y, pose.heading_deg + dh, x + gx * math.cos(a) - gy * math.sin(a),
                    y + gx * math.sin(a) + gy * math.cos(a), pose.side_mm, pose.t)

    def _stalled(self, now, pose, l, r):
        """Diagnostic only: wheels commanded for stall_window_s but the tag has not moved
        (wall contact, wheel slip, or turning below the torque needed to scrub the tyres).
        Returns None, 'drive' or 'spin'. Does not change any command."""
        if self._stuck_pulses >= 3:
            return 'spin'                   # three turn pulses in a row did not turn the robot
        if pose is None or now - pose.t > self.o['pose_timeout_s']:
            self.motion = []
            return None
        self.motion.append((now, pose.x, pose.y, pose.heading_deg, l, r))
        window = self.o.get('stall_window_s', 0.6)
        while len(self.motion) > 1 and now - self.motion[1][0] >= window:
            self.motion.pop(0)
        t0, x0, y0, h0, _, _ = self.motion[0]
        if now - t0 < window or any(abs(a) + abs(b) < 0.2 for *_, a, b in self.motion):
            return None
        spin = all(a * b < 0 for *_, a, b in self.motion)
        moved = math.hypot(pose.x - x0, pose.y - y0)
        turned = abs(math.degrees(wrap(math.radians(pose.heading_deg - h0))))
        if spin:
            return 'spin' if turned < 5 else None
        return 'drive' if moved < 15 and turned < 5 else None

    def _step(self, now, pose, targets, observations, status=None):
        """Returns (l, r, events). events: [(cmd, fields)] one-off commands to send."""
        ev = []
        if status and status.get('state') not in (None, 'RUNNING'):
            self.wall.pause(now, 'wall_wait_firmware')
            self.debug['reason'] = 'firmware_' + str(status.get('state'))
            return 0.0, 0.0, ev
        if pose is None or not 0 <= now - pose.t <= min(self.o['pose_timeout_s'], self.wall.o['wall_pose_max_age_s']):
            self.wall.pause(now, 'wall_wait_fresh_tag')
            self.debug['reason'] = 'tag_missing' if pose is None else 'pose_stale'
            return 0.0, 0.0, ev                          # no fresh pose: stand still
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.heading_deg, pose.grip_x, pose.grip_y)):
            self.wall.pause(now, 'wall_invalid_pose')
            self.debug['reason'] = 'pose_invalid'
            return 0.0, 0.0, ev
        # GRIP/RELEASE are stationary: finish the servo operation before moving a payload.
        if self.state not in ('GRIP', 'RELEASE'):
            recovery = self._recover_wall(now, pose, observations)
            if recovery is not None:
                return recovery
        if observations:
            xs = [o['x'] for o in observations]
            ys = [o['y'] for o in observations]
            self.pile_center = (sorted(xs)[len(xs) // 2], sorted(ys)[len(ys) // 2])
        if self.o['predict_pose']:
            pose = self._predict(pose)
        o, s = self.o, self.state
        usable = [t for t in targets if self._usable(t, now)]
        if not usable and o['skip_alone_s'] is not None:
            # every stone on offer is being skipped: retry sooner rather than park (V2)
            usable = [t for t in targets if self._usable(t, now, o['skip_alone_s'])]
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
            if self._choose(pose)(usable) or self._elapsed(now) > o['timeouts_s']['PARK']:
                self._go('SEARCH', now)
                return 0.0, 0.0, ev
            if math.hypot(self.park[0] - pose.x, self.park[1] - pose.y) < 60:
                return 0.0, 0.0, ev
            return (*self._drive_to(pose, pose.x, pose.y, self.park[0], self.park[1], o['cruise']), ev)

        if s in ('GOTO_STAGE', 'ALIGN') and self._servo_at(status, 'grip', o['grip_open']):
            jaw = self._jaw_stone(pose, observations, now)
            if jaw:
                self.lock.target = dict(jaw)            # a Jaw stone beats the locked stone
                self._start_grip(jaw, now, ev, f"jaw stone colour {jaw['color']}")
                return 0.0, 0.0, ev

        if s == 'GOTO_STAGE':
            locked = self.lock.target
            covered = bool(locked and self._under_robot(pose, locked['x'], locked['y']))
            t = self.lock.update(usable, now, observations, occluded=covered, acquire=False)
            self.debug['target_under_robot'] = covered
            if t is None:
                self._go('SEARCH', now, f'target {self.lock.reason}')
                return 0.0, 0.0, ev
            if not self._approach_inside(t, self.heading):
                self._skip_target(now, 'approach crosses wall margin')
                self._go('SEARCH', now, 'approach crosses wall margin')
                return 0.0, 0.0, ev
            sx, sy = self._centre_for(t, self.heading, o['stage_mm'])   # heading frozen at lock
            ux, uy = math.cos(self.heading), math.sin(self.heading)
            gx, gy = t['x'] - pose.grip_x, t['y'] - pose.grip_y
            along, side = gx * ux + gy * uy, -gx * uy + gy * ux
            # If already closer than the staging distance, align and approach;
            # do not turn back toward a staging point behind the robot.
            on_line = (-o['grip_tol_mm'] <= along < o['stage_mm'] + 60 and abs(side) < 12
                       and abs(wrap(self.heading - heading)) < math.radians(25))
            ax, ay = self._axle(pose)
            self.debug.update(goal_x_mm=sx, goal_y_mm=sy,
                              goal_distance_mm=math.hypot(sx-ax, sy-ay),
                              along_mm=along, side_mm=side)
            if on_line or self._arrived(pose, ax, ay, sx, sy, o['stage_tol_mm']):
                self._go('ALIGN', now)
                return 0.0, 0.0, ev
            if self._elapsed(now) > o['timeouts_s']['GOTO_STAGE']:
                self._skip_target(now, 'stage timeout')
                self._go('BACKOFF', now, 'stage timeout')
                return 0.0, 0.0, ev
            return (*self._drive_to(pose, ax, ay, sx, sy, o['cruise']), ev)

        if s == 'ALIGN':
            t = self.lock.update(usable, now, observations, occluded=True, acquire=False)
            if t is None:
                self._go('SEARCH', now, f'target {self.lock.reason}')
                return 0.0, 0.0, ev
            cmd = self._turn_to(pose, self.heading)
            ready = self._servo_at(status, 'grip', o['grip_open'])
            if cmd is None and not ready:
                self.debug['reason'] = 'waiting_gripper_open'
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
            t = self.lock.update(usable, now, observations, occluded=True, acquire=False)
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
            self.debug.update(along_mm=along, side_mm=side)
            if along <= o['grip_tol_mm']:
                if abs(side) > o['approach_max_side_mm']:
                    if self.retries < 1:                 # back up and line up once more
                        self.retries += 1
                    else:
                        self._skip_target(now, 'missed sideways')
                    self._go('BACKOFF', now, f'side error {side:.0f} mm')
                    return 0.0, 0.0, ev
                self._start_grip(t, now, ev)
                return 0.0, 0.0, ev
            if self._elapsed(now) > o['timeouts_s']['APPROACH']:
                self._skip_target(now, 'approach timeout')
                self._go('BACKOFF', now, 'approach timeout')
                return 0.0, 0.0, ev
            wanted = self.heading + math.atan2(side, o.get('approach_lookahead_mm', 150))
            err = wrap(wanted - heading)
            self.debug['heading_error_deg'] = math.degrees(err)
            if self._pulse is not None or abs(err) > math.radians(20):   # badly off: turn first
                cmd = self._turn_step(err, pose, 20)
                if cmd is not None:
                    self.debug['reason'] = 'turn_to_approach'
                    return (*cmd, ev)
            v = o['creep']
            limit = min(.12, abs(v) * clamp(o['max_forward_steer_ratio'], 0, .95))
            steer = clamp(0.8 * err, -limit, limit)
            self.debug['reason'] = 'creep_to_stone'
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
            wx, wy = self._carry_waypoint(pose.grip_x, pose.grip_y, zx, zy)
            return (*self._drive_to(pose, pose.grip_x, pose.grip_y, wx, wy, o['cruise']), ev)

        if s == 'DISCARD':
            dx, dy = self.discard_to
            if self._in_any_zone(pose.grip_x, pose.grip_y, 90) and self._elapsed(now) < 15:
                return (*self._drive_to(pose, pose.grip_x, pose.grip_y, dx, dy, o['cruise']), ev)
            ev.append(('grip', {'p': 'open'}))
            self._go('BACKOFF', now, 'discarded')
            return 0.0, 0.0, ev

        if s == 'RELEASE':
            if self._servo_done(now, status, 'grip', o['grip_open']):
                self.released += 1
                self.carrying = None
                self.lock.done()
                self._go('BACKOFF', now, 'released')
            return 0.0, 0.0, ev

        if s == 'BACKOFF':
            if self.backoff_from is None or self._elapsed(now) < 0.05:
                self.backoff_from = (pose.x, pose.y)
            moved = math.hypot(pose.x - self.backoff_from[0], pose.y - self.backoff_from[1])
            self.debug['backoff_mm'] = moved
            if moved >= o['backoff_mm'] or self._elapsed(now) > o['backoff_s']:
                self.backoff_from = None
                self._go('SEARCH', now)
                return 0.0, 0.0, ev
            return -o['creep'], -o['creep'], ev

        return 0.0, 0.0, ev


def grip_calibration_warning(cfg):
    tag = cfg.get('robot_tag', {})
    missing = []
    if not tag.get('grip_calibrated'):
        missing.append('grip offset: python calibrate_grip.py <camera> --write  (stone in closed jaws, arm down)')
    if 'axle_offset_mm' not in tag:
        missing.append('axle offset: python calibrate_grip.py <camera> --axle --write  (spin on the spot)')
    return ('WARNING, not measured yet:\n  ' + '\n  '.join(missing)) if missing else None


def apply_overrides(cfg, items):
    """--set KEY=VALUE: replace calib.json "autonomy" values for this run. Values are JSON
    (0.5, true, null, {"ALIGN": 8}); anything else stays a string (turn_mode=fixed).
    --set vision.KEY=VALUE sets one of the named vision switches (profiles.VISION_SWITCHES)."""
    from profiles import VISION_SWITCHES
    for item in items:
        key, sep, value = item.partition('=')
        section, dot, name = key.partition('.')
        if sep and dot and section == 'vision' and name in VISION_SWITCHES:
            try:
                value = json.loads(value)
            except ValueError:
                pass
            valid, expected = VISION_SWITCHES[name]
            if not valid(value):
                raise ValueError(f'--set {item}: vision.{name} must be {expected}')
            cfg.setdefault('vision', {})[name] = value
            continue
        if not sep or key not in DEFAULTS:
            raise ValueError(f'--set {item}: expected KEY=VALUE with KEY one of the autonomy options '
                             f'({", ".join(sorted(DEFAULTS))}) or vision.KEY with KEY one of '
                             f'({", ".join(sorted(VISION_SWITCHES))})')
        try:
            value = json.loads(value)
        except ValueError:
            pass
        cfg.setdefault('autonomy', {})[key] = value


def prepare_config(cfg, profile, overrides):
    """In memory only: the profile's switches (profiles.py), then --set (which wins)."""
    import profiles
    profiles.apply_profile(cfg, profile)
    apply_overrides(cfg, overrides)
    return cfg


def min_duty_problem(cfg):
    floor = cfg.get('autonomy', {}).get('min_duty')
    if floor is not None and (isinstance(floor, bool) or not isinstance(floor, (int, float))
                              or not 0 <= floor <= 1):
        return f'autonomy.min_duty must be a number from 0 to 1, or null (got {floor!r})'
    return None


def drive_floor(o):
    """Extra drive-packet fields for the min_duty option: {} = firmware MIN_DUTY."""
    return {} if o.get('min_duty') is None else {'m': round(float(o['min_duty']), 3)}


# ------------------------------------------------------------ simulation runner
def run_sim(cfg, stones, seconds=300.0, start=None, params=None, seed=0, show=False, rate=30.0):
    import sim
    w, h = cfg['arena']['size_mm']
    # Start ordinary mission demos in the navigable interior. Explicit edge starts
    # still exercise wall recovery (the previous 0.92*w start was in the wall margin).
    x, y, hd = start or (w * 0.85, h * 0.5, 180.0)
    robot = sim.SimRobot(cfg, stones, x, y, hd, params=params, seed=seed)
    planner = Planner(cfg)
    if 'camera_delay_s' not in cfg.get('autonomy', {}):
        # predict over the simulated robot's own delay, as camera_delay_s is measured on the real one
        planner.o['camera_delay_s'] = robot.p['latency_s'] + 0.5 / rate
    floor = drive_floor(planner.o)
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
        robot.command('drive', t, l=l, r=r, **floor)         # 50 Hz, like the sender thread
        t += dt
    return {**robot.score(), 'released_by_planner': planner.released, 'time_s': round(t, 1),
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

    def __init__(self, link, rate=20.0, stale_s=0.25, floor=None):
        self.link, self.rate, self.stale_s = link, rate, stale_s
        self.floor = floor or {}            # drive_floor(): the min_duty field, if any
        self.l = self.r = 0.0
        self.t = 0.0
        self.valid_until = None
        self.ok = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set(self, l, r, valid_until=None):
        with self.lock:
            self.l, self.r, self.t = l, r, time.monotonic()
            self.valid_until = valid_until

    def event(self, cmd, **fields):
        # The heartbeat and one-shot commands share a sequence counter. Serialize
        # sends so a newer drive packet cannot overtake a gripper/start command.
        with self.lock:
            return self.link.send(cmd, **fields)

    def _run(self):
        while self.ok:
            with self.lock:
                now = time.monotonic()
                fresh = now - self.t <= self.stale_s and (self.valid_until is None or now < self.valid_until)
                l, r = (self.l, self.r) if fresh else (0.0, 0.0)
                self.link.send('drive', l=round(l, 3), r=round(r, 3), **self.floor)
            time.sleep(1.0 / self.rate)

    def stop(self):
        self.ok = False
        self.thread.join(timeout=1.0)


def setup_problems(cfg, config_path):
    """Check the actual run configuration before opening a camera or enabling wheels."""
    problems = []
    arena = cfg.get('arena', {})
    if len(arena.get('corners_px', [])) != 4:
        problems.append('arena.corners_px needs four calibrated corners')
    # zone_colors: the scoring zones on this field (default all six; a practice field may
    # have fewer). Stones need HSV ranges for every zone colour and every aliased colour.
    from find_zones import expected_zone_colors
    zone_colors = expected_zone_colors(cfg)
    alias = {int(k): int(v) for k, v in (cfg.get('autonomy', {}).get('color_alias') or {}).items()}
    for src, dst in sorted(alias.items()):
        if dst not in zone_colors or src in zone_colors:
            problems.append(f'color_alias {src}->{dst}: the target must be a zone on this field '
                            f'{zone_colors} and the source must not have its own zone')
    missing = [str(cid) for cid in sorted(set(zone_colors) | set(alias))
               if not any(str(k).split('_')[0] == str(cid) and bool(v)
                          for k, v in cfg.get('hsv', {}).items())]
    if missing:
        problems.append('missing HSV color IDs: ' + ', '.join(missing))
    zones = cfg.get('zones', {})
    if sorted(int(str(k).split('_')[0]) for k in zones) != zone_colors:
        problems.append(f'zones must contain exactly the labeled scoring destinations {zone_colors} '
                        '(run find_zones.py; set "zone_colors" for a field with fewer zones)')
    for key, zone in zones.items():
        if len(zone.get('center_mm', [])) != 2 or zone.get('radius_mm', 0) <= 0:
            problems.append(f'invalid zone geometry: {key}')
    if not cfg.get('robot_tag'):
        problems.append('robot_tag configuration is missing')
    for key in ('camera_height_mm', 'camera_floor_xy_mm'):
        if key in cfg.get('robot_tag', {}) and cfg['robot_tag'][key] is None:
            problems.append(f'robot_tag.{key} not measured yet (null): floor to lens height / '
                            'floor point under the lens in field mm (see minifield/README.md)')
    background_path = Path(config_path).parent / cfg.get('background_path', 'background.png')
    if not background_path.is_file():
        problems.append('missing empty-field reference: ' + str(background_path))
    return problems


def diagnostic_text(info):
    def number(key, suffix=''):
        value = info.get(key)
        return '--' if value is None else f'{value:.1f}{suffix}'
    return (f"{info['state']} | {info['reason']} | L={info['l']:+.2f} R={info['r']:+.2f} | "
            f"goal={number('goal_distance_mm', 'mm')} err={number('heading_error_deg', 'deg')} | "
            f"lock={info['lock_reason']}" + (f" | STALLED {info['stalled']}" if info.get('stalled') else ''))


def run_real(args, cfg):
    import cv2
    from detect_live import LatestFrame
    from perception import Perception, draw_robot
    from teleop import Link
    problems = setup_problems(cfg, args.config)
    if problems:
        raise SystemExit('Autonomy not started. Check ' + str(args.config.resolve()) + ':\n- ' + '\n- '.join(problems))
    warning = grip_calibration_warning(cfg)
    if warning:
        print(warning)
    background_path = args.config.parent / cfg.get('background_path', 'background.png')
    background = cv2.imread(str(background_path)) if background_path.exists() else None
    if background is None:
        raise SystemExit('No background.png: run calibrate_arena.py first')
    perception = Perception(cfg, background)
    planner = Planner(cfg)
    profile = cfg.get('profile', 'v1')
    run_dir = Path(args.log_dir) / (time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
                                    + ('' if profile == 'v1' else '-' + profile))
    run_dir.mkdir(parents=True)
    (run_dir / 'config.json').write_text(json.dumps(cfg, indent=2), encoding='utf-8')
    cap = cv2.VideoCapture(args.camera if args.camera is not None else cfg.get('camera_index', 0))
    if not cap.isOpened():
        raise SystemExit('Cannot open camera')
    from camera_io import apply_camera_properties
    apply_camera_properties(cap, cfg)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    reader = LatestFrame(cap)
    dry_run = getattr(args, 'dry_run', False)
    floor = drive_floor(planner.o)
    link = None if dry_run else Link(args.esp_ip, args.port)
    sender = None if dry_run else DriveSender(link, floor=floor)
    per_px = float(cfg['arena'].get('mm_per_px', 2))
    last_print = 0.0
    print('Config:', args.config.resolve())
    print('Dry run (no robot commands).' if dry_run else 'Real robot control enabled.')
    print(f"Drive floor: min_duty {floor['m']} (sent with every drive packet)" if floor
          else 'Drive floor: firmware MIN_DUTY')
    print('Diagnostics:', run_dir / 'trace.jsonl')
    writer, video_frame = None, -1
    if getattr(args, 'record', False):
        # MJPG in AVI: every frame is a complete JPEG, so the file stays readable even if the
        # program is killed before it is closed (an unclosed .mp4 has no index and won't open).
        print('Recording:', run_dir / 'video.avi', '(raw frames; trace video_frame = frame index)')
    try:
        with (run_dir / 'trace.jsonl').open('w', encoding='utf-8') as trace:
            if sender:
                sender.event('start')
            while True:
                ok, raw = reader.read()
                if not ok:
                    break
                now = time.monotonic()
                if link:
                    link.poll()
                if getattr(args, 'record', False):
                    if writer is None:
                        h, w = raw.shape[:2]
                        writer = cv2.VideoWriter(str(run_dir / 'video.avi'), cv2.VideoWriter_fourcc(*'MJPG'),
                                                 args.record_fps, (w, h))
                        writer.set(cv2.VIDEOWRITER_PROP_QUALITY, 80)
                    writer.write(raw)
                    video_frame += 1
                snap = perception.step(raw, now)
                decision_t = time.monotonic()
                status = link.status if link and link.status_age() < 1.0 else None
                if floor and status and 'min_duty' not in status:
                    # old firmware ignores "m" and would silently drive at its own MIN_DUTY
                    print('Firmware does not report min_duty: flash firmware/robot_ctrl, '
                          'or run without min_duty. Stopping.')
                    break
                l, r, events = planner.step(decision_t, snap.pose, snap.targets, snap.observations, status,
                                            perception_status=snap.status, require_status=not dry_run)
                if sender:
                    sender.set(l, r, valid_until=planner.debug.get('wall_command_until'))
                    for cmd, fields in events:
                        sender.event(cmd, **fields)
                info = dict(planner.debug, t=decision_t, dry_run=dry_run, events=events,
                            frame_ms=(decision_t-now)*1000,
                            video_frame=video_frame if writer is not None else None,
                            firmware=status, pose=snap.pose.as_dict() if snap.pose else None,
                            observation_list=snap.observations, target_list=snap.targets,
                            tag_reason=perception.pose_est.last_reason if perception.pose_est else 'not configured')
                trace.write(json.dumps(info) + '\n')
                if decision_t-last_print >= .5:
                    print(diagnostic_text(info), flush=True)
                    trace.flush()
                    last_print = decision_t
                if not args.headless:
                    import numpy as np
                    frame = draw_robot(snap.frame, snap, per_px)
                    if planner.lock.target:
                        t = planner.lock.target
                        cv2.circle(frame, (round(t['x'] / per_px), round(t['y'] / per_px)), 14, (0, 0, 255), 2)
                    if info.get('goal_x_mm') is not None:
                        goal = (round(info['goal_x_mm']/per_px), round(info['goal_y_mm']/per_px))
                        cv2.drawMarker(frame, goal, (0, 165, 255), cv2.MARKER_CROSS, 20, 2)
                    h, w = frame.shape[:2]
                    canvas = np.full((h+116, max(w, 1000), 3), 25, np.uint8)
                    canvas[:h, :w] = frame
                    state = 'DRY RUN' if dry_run else status.get('state', '?') if status else 'NO LINK'
                    fw_floor = status.get('min_duty') if status else None
                    lines = [f'{profile.upper()} | {state} | vision={snap.status} | released={planner.released} | floor='
                             + ('--' if fw_floor is None else f'{fw_floor:.2f}'), diagnostic_text(info),
                             f"frame={info['frame_ms']:.0f}ms | tag={info['tag_reason']} | orange cross=drive goal | q/x/ESC stop"]
                    if warning:
                        lines.append('grip/axle offset not calibrated')
                    for i, line in enumerate(lines):
                        cv2.putText(canvas, line, (10, h+22+i*25), 0, .45, (240, 240, 240), 1)
                    cv2.imshow('autonomy', canvas)
                    if getattr(args, 'show_full_frame', False):
                        raw_view = raw.copy()
                        corners = np.array(cfg['arena']['corners_px'], np.int32)
                        cv2.polylines(raw_view, [corners], True, (0, 200, 255), 2)
                        tag_corners = perception.pose_est.last_corners_px if perception.pose_est else None
                        if snap.pose is not None and tag_corners is not None:
                            cv2.polylines(raw_view, [np.round(tag_corners).astype(np.int32)], True, (255, 0, 255), 2)
                        cv2.putText(raw_view, info['state'] + ' | ' + str(info.get('wall_reason', '')),
                                    (10, 24), 0, .55, (255, 0, 255), 2)
                        cv2.imshow('full camera (yellow = calibrated field)', raw_view)
                    if (cv2.waitKey(1) & 255) in (ord('q'), 27, ord('x')):
                        break
                if status and status.get('state') == 'IDLE' and status.get('why') == 'remote stop':
                    break
    finally:
        # Each step on its own: a failed network send (Wi-Fi gone) must not skip the others.
        def attempt(what, fn):
            try:
                fn()
            except Exception as exc:          # keep shutting down; report and continue
                print(f'shutdown: {what} failed: {exc}')
        if sender:
            attempt('stop sender', sender.stop)
            attempt('stop wheels', lambda: (link.send('drive', l=0, r=0), link.send('stop')))
            attempt('close link', link.sock.close)
        if writer is not None:
            attempt('close video', writer.release)
        attempt('stop camera reader', reader.stop)
        attempt('release camera', cap.release)
        attempt('close windows', cv2.destroyAllWindows)
        print('released', planner.released, 'events:', planner.events_log[-10:])


def main(profile='v1'):
    import profiles
    p = argparse.ArgumentParser(description=__doc__ if profile == 'v1' else profiles.__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip', nargs='?')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--camera', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--headless', action='store_true')
    p.add_argument('--show-full-frame', action='store_true', help='also show the uncropped camera and calibrated boundary')
    p.add_argument('--dry-run', action='store_true', help='Live camera/planner preview, no UDP or robot commands')
    p.add_argument('--check-config', action='store_true', help='Check calibration files without camera or robot')
    p.add_argument('--record', action='store_true',
                   help='Save the processed raw camera frames to video.avi next to trace.jsonl (replay with detect_live.py --video)')
    p.add_argument('--record-fps', type=float, default=30,
                   help='Playback rate written into video.avi (the loop runs ~30 fps); exact timing is in trace.jsonl')
    p.add_argument('--log-dir', type=Path, default=Path('runs/autonomy'), help='Config and per-frame decision logs')
    p.add_argument('--sim', action='store_true', help='simulated robot and field')
    p.add_argument('--scenario', choices=['pile', 'scattered'], default='pile')
    p.add_argument('--show', action='store_true', help='draw the simulation')
    p.add_argument('--seconds', type=float, default=300)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--noise', action='store_true', help='sim: pose noise, dropped frames, latency, failed grabs')
    p.add_argument('--field-physics', nargs='?', const='charged', choices=['charged', 'low-battery'],
                   help='sim: MIN_DUTY, turn behaviour, walls and tag loss near edges as measured on the field')
    p.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                   help='override a calib.json "autonomy" value for this run, e.g. --set min_duty=0.5 '
                        '--set creep=0.25, or a vision switch: --set vision.pile_edge_pixels=nearest '
                        '(saved in the run folder config.json; calib.json is not changed)')
    args = p.parse_args()
    if not args.config.is_file():
        p.error(f'Config not found: {args.config}. Pass --config with your actual calibrated JSON file.')
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    try:
        prepare_config(cfg, profile, args.set)
    except ValueError as exc:
        p.error(str(exc))
    print('Version:', profiles.TITLES[profile])
    if min_duty_problem(cfg):
        p.error(min_duty_problem(cfg))
    if args.check_config:
        problems = setup_problems(cfg, args.config)
        print('Config:', args.config.resolve())
        if problems:
            raise SystemExit('Not ready:\n- ' + '\n- '.join(problems))
        print('Required configuration and background file present; field alignment still needs a live check.')
        return
    if args.sim:
        params = ({'pose_noise_mm': 4, 'heading_noise_deg': 1.5, 'tag_dropout': 0.1,
                   'latency_s': 0.12, 'grip_success': 0.85} if args.noise else {})
        if args.field_physics:
            import sim
            params.update(sim.FIELD_PARAMS if args.field_physics == 'charged' else sim.LOW_BATTERY_PARAMS)
        result = run_sim(cfg, scenario(cfg, args.scenario, args.seed), args.seconds, params=params,
                         seed=args.seed, show=args.show)
        for entry in result['log']:
            print(entry)
        print(f"correct {result['correct']}  wrong {result['wrong']}  in {result['time_s']} s "
              f"-> score {5 * result['correct'] - result['wrong']}")
        return
    if not args.esp_ip and not args.dry_run:
        p.error('esp_ip required (or use --sim / --dry-run)')
    run_real(args, cfg)


if __name__ == '__main__':
    main()
