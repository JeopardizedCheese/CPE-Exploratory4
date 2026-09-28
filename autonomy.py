"""Autonomous run: pick one stone, carry it to its colour's zone, repeat.

    python autonomy.py --sim --show                 # simulated robot and field, watch it
    python autonomy.py --sim --scenario scattered   # easier field
    python autonomy.py 10.178.188.50 --camera 1     # real robot (sends 'start'; q/x/ESC stops)
    python autonomy.py --dry-run --camera 1       # camera/planner only, no robot commands
    python autonomy.py 10.178.188.50 --camera 1 --record   # also save video.mp4 for replay

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

from target_lock import TargetLock

DEFAULTS = {
    'cruise': 0.45, 'creep': 0.18, 'turn': 0.35, 'min_turn': 0.16,
    'kp_turn': 1.0,                 # drive-command per radian of heading error (fixed-power turns)
    'max_forward_steer_ratio': 0.5, # below 1: neither wheel reverses in the forward branch; with
                                    # MIN_DUTY, 0.8 made 'forward' nearly a pivot (overshoot, hunting)
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
    # Turning in place is closed-loop on the turn rate measured from the tag: the tyres need
    # about 0.8 duty to start turning but much less to keep turning, so any fixed power
    # either stalls or whips round and overshoots. Power rises while the robot does not turn
    # and drops as soon as it turns faster than wanted. false = old fixed-power turns.
    'turn_rate_control': True,
    'turn_rate_max_dps': 60,         # wanted turn rate far from the goal heading
    'turn_rate_min_dps': 20,         # wanted turn rate close to it
    'turn_rate_per_deg': 2.0,        # wanted deg/s per degree of heading error
    'turn_power_max': 0.6,           # never push harder than this to break free
    'turn_boost_per_s': 0.4,         # power rise per second while turning too slowly
    'turn_breakaway_cut': 0.6,       # once it starts turning, drop power to this fraction
    'turn_ease_per_s': 1.2,          # power drop per second while turning too fast
    'turn_lead_s': 0.12,             # camera + processing delay: stop turning this early
    'turn_exit_deg': 15,             # a turn toward a drive goal continues until this close
                                     # (hysteresis against turn_in_place_deg: no turn/drive dance)
    'turn_keep_dir_deg': 150,        # goal almost behind: keep the current turn direction
    'backout_inside_reach': True,    # CARRY goal inside the grip point's turning circle: creep
                                     # straight back a few cm instead of turning on the spot forever
    'kp_drive': 0.5,                 # steering per radian while driving forward
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
        self.released = 0
        self.events_log = []
        self.pile_center = None
        self.motion = []                    # (t, x, y, heading_deg, l, r) for stall diagnostics
        self.turn_power = None              # last spin power that turned the robot (learned)
        self._spin_dir = 0.0
        self._spin_power = 0.0
        self._spin_moving = False
        self._was_spinning = False          # _drive_to: previous frame turned in place
        self.turn_t = None                  # time of the last spin command
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

    def _usable(self, t, now):
        if t['color'] not in self.zones:
            return False
        return not any(now < until and math.hypot(t['x'] - x, t['y'] - y) < self.o['skip_mm']
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
        self.debug.update(goal_x_mm=gx, goal_y_mm=gy, goal_distance_mm=dist,
                          heading_error_deg=math.degrees(err))
        turning = o['turn_rate_control'] and self._was_spinning
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
        if turning:
            left = err - math.radians(self._yaw_rate() * o['turn_lead_s'])   # after the camera delay
            turning = abs(left) > math.radians(o['turn_exit_deg']) and left * err > 0
        if abs(err) > math.radians(o['turn_in_place_deg']) or turning:
            self.debug['reason'] = 'turn_to_goal'
            self._was_spinning = True
            return self._spin(err, pose.t)
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
        lead = math.radians(self._yaw_rate() * self.o['turn_lead_s']) if self.o['turn_rate_control'] else 0.0
        if abs(err) <= math.radians(self.o['align_tol_deg']) or \
                (err * lead > 0 and abs(err - lead) <= math.radians(self.o['align_tol_deg'])):
            return None                     # there, or still coasting there within the camera delay
        return self._spin(err, pose.t)

    def _yaw_rate(self, window=0.25):
        """Measured turn rate (deg/s, + = heading increasing) from recent fresh poses."""
        if not self.motion:
            return 0.0
        end = self.motion[-1]
        pts = [m for m in self.motion if end[0] - m[0] <= window]
        if end[0] - pts[0][0] < 0.08:
            return 0.0
        return math.degrees(wrap(math.radians(end[3] - pts[0][3]))) / (end[0] - pts[0][0])

    def _spin(self, err, now):
        """Turn in place towards heading error err (rad): returns (l, r)."""
        o = self.o
        if not o['turn_rate_control']:
            w = math.copysign(max(o['min_turn'], min(o['turn'], o['kp_turn'] * abs(err))), err)
            return w, -w
        err_deg = abs(math.degrees(err))
        wanted = clamp(o['turn_rate_per_deg'] * err_deg, o['turn_rate_min_dps'], o['turn_rate_max_dps'])
        rate = self._yaw_rate() * math.copysign(1, err)       # + = turning the right way
        dt = 0.0 if self.turn_t is None else clamp(now - self.turn_t, 0.0, 0.2)
        if self.turn_t is None or now - self.turn_t > 0.5 or self._spin_dir * err < 0:
            # new turn: start a little below what last broke the robot free
            power = o['min_turn'] if self.turn_power is None else max(o['min_turn'], self.turn_power - 0.05)
            self._spin_moving = False
        else:
            power = self._spin_power
            if not self._spin_moving and rate > 0.5 * wanted:
                # Broke free. The camera saw it ~turn_lead_s late, and power kept rising
                # meanwhile: remember the power at the break, and cut, because keeping
                # a turn going takes much less than starting it.
                self._spin_moving = True
                self.turn_power = max(o['min_turn'], power - o['turn_boost_per_s'] * o['turn_lead_s'])
                power *= o['turn_breakaway_cut']
            elif rate < 0.5 * wanted:
                power += o['turn_boost_per_s'] * dt
                if rate < 5:
                    self._spin_moving = False   # stuck again: the next start is a new break
            elif rate > 1.3 * wanted:
                power -= o['turn_ease_per_s'] * dt
        power = clamp(power, o['min_turn'], o['turn_power_max'])
        self._spin_power, self.turn_t, self._spin_dir = power, now, math.copysign(1, err)
        self.debug.update(turn_power=power, turn_rate_dps=rate, turn_wanted_dps=wanted)
        w = math.copysign(power, err)
        return w, -w

    # ------------------------------------------------------------ main step
    def step(self, now, pose, targets, observations, status=None, *,
             perception_status='ok', require_status=False):
        """Plan one frame; diagnostics explain both commands and stop conditions.

        Real runs require fresh firmware status and valid perception. Simulation
        callers may omit status; neither a lost tag nor invalid vision is bypassed.
        """
        before = self.state
        self.debug = {'state': before, 'reason': '', 'targets': len(targets),
                      'observations': len(observations), 'perception': perception_status,
                      'pose_age_s': None if pose is None else now-pose.t,
                      'firmware_state': status.get('state') if status else None}
        if perception_status != 'ok':
            result = (0.0, 0.0, [])
            self.debug['reason'] = 'vision_' + perception_status
        elif require_status and not status:
            result = (0.0, 0.0, [])
            self.debug['reason'] = 'no_fresh_firmware_status'
        else:
            result = self._step(now, pose, targets, observations, status)
        l, r, events = result
        if self.state != before:
            why = self.events_log[-1][3] if self.events_log else ''
            self.debug['reason'] = f'{before}->{self.state}' + (f': {why}' if why else '')
        if not self.debug['reason']:
            self.debug['reason'] = 'waiting_' + self.state.lower()
        self.debug.update(state=self.state, l=l, r=r, lock_reason=self.lock.reason,
                          target=dict(self.lock.target) if self.lock.target else None,
                          stalled=self._stalled(now, pose, l, r))
        return result

    def _stalled(self, now, pose, l, r):
        """Diagnostic only: wheels commanded for stall_window_s but the tag has not moved
        (wall contact, wheel slip, or turning below the torque needed to scrub the tyres).
        Returns None, 'drive' or 'spin'. Does not change any command."""
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
            self.debug['reason'] = 'firmware_' + str(status.get('state'))
            return 0.0, 0.0, ev
        if pose is None or now - pose.t > self.o['pose_timeout_s']:
            self.debug['reason'] = 'tag_missing' if pose is None else 'pose_stale'
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
            if abs(err) > math.radians(20):                  # badly off: turn on the spot first
                self.debug['reason'] = 'turn_to_approach'
                return (*self._spin(err, pose.t), ev)
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

    def event(self, cmd, **fields):
        # The heartbeat and one-shot commands share a sequence counter. Serialize
        # sends so a newer drive packet cannot overtake a gripper/start command.
        with self.lock:
            return self.link.send(cmd, **fields)

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


def setup_problems(cfg, config_path):
    """Check the actual run configuration before opening a camera or enabling wheels."""
    problems = []
    arena = cfg.get('arena', {})
    if len(arena.get('corners_px', [])) != 4:
        problems.append('arena.corners_px needs four calibrated corners')
    missing = [str(cid) for cid in range(1, 7)
               if not any(str(k).split('_')[0] == str(cid) and bool(v)
                          for k, v in cfg.get('hsv', {}).items())]
    if missing:
        problems.append('missing HSV color IDs: ' + ', '.join(missing))
    zones = cfg.get('zones', {})
    if {str(k).split('_')[0] for k in zones} != {str(c) for c in range(1, 7)}:
        problems.append('zones must contain all six labeled scoring destinations')
    for key, zone in zones.items():
        if len(zone.get('center_mm', [])) != 2 or zone.get('radius_mm', 0) <= 0:
            problems.append(f'invalid zone geometry: {key}')
    if not cfg.get('robot_tag'):
        problems.append('robot_tag configuration is missing')
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
    run_dir = Path(args.log_dir) / (time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6])
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
    link = None if dry_run else Link(args.esp_ip, args.port)
    sender = None if dry_run else DriveSender(link)
    per_px = float(cfg['arena'].get('mm_per_px', 2))
    last_print = 0.0
    print('Config:', args.config.resolve())
    print('Dry run (no robot commands).' if dry_run else 'Real robot control enabled.')
    print('Diagnostics:', run_dir / 'trace.jsonl')
    writer, video_frame = None, -1
    if getattr(args, 'record', False):
        print('Recording:', run_dir / 'video.mp4', '(raw frames; trace video_frame = frame index)')
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
                        writer = cv2.VideoWriter(str(run_dir / 'video.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                                 args.record_fps, (w, h))
                    writer.write(raw)
                    video_frame += 1
                snap = perception.step(raw, now)
                decision_t = time.monotonic()
                status = link.status if link and link.status_age() < 1.0 else None
                l, r, events = planner.step(decision_t, snap.pose, snap.targets, snap.observations, status,
                                            perception_status=snap.status, require_status=not dry_run)
                if sender:
                    sender.set(l, r)
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
                    lines = [f'{state} | vision={snap.status} | released={planner.released}', diagnostic_text(info),
                             f"frame={info['frame_ms']:.0f}ms | tag={info['tag_reason']} | orange cross=drive goal | q/x/ESC stop"]
                    if warning:
                        lines.append('grip/axle offset not calibrated')
                    for i, line in enumerate(lines):
                        cv2.putText(canvas, line, (10, h+22+i*25), 0, .45, (240, 240, 240), 1)
                    cv2.imshow('autonomy', canvas)
                    if (cv2.waitKey(1) & 255) in (ord('q'), 27, ord('x')):
                        break
                if status and status.get('state') == 'IDLE' and status.get('why') == 'remote stop':
                    break
    finally:
        if sender:
            sender.stop()
            link.send('drive', l=0, r=0)
            link.send('stop')
            link.sock.close()
        reader.stop()
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
        print('released', planner.released, 'events:', planner.events_log[-10:])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip', nargs='?')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--camera', type=int)
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    p.add_argument('--headless', action='store_true')
    p.add_argument('--dry-run', action='store_true', help='Live camera/planner preview, no UDP or robot commands')
    p.add_argument('--check-config', action='store_true', help='Check calibration files without camera or robot')
    p.add_argument('--record', action='store_true',
                   help='Save the processed raw camera frames to video.mp4 next to trace.jsonl (replay with detect_live.py --video)')
    p.add_argument('--record-fps', type=float, default=15, help='Playback rate written into video.mp4; timing is in trace.jsonl')
    p.add_argument('--log-dir', type=Path, default=Path('runs/autonomy'), help='Config and per-frame decision logs')
    p.add_argument('--sim', action='store_true', help='simulated robot and field')
    p.add_argument('--scenario', choices=['pile', 'scattered'], default='pile')
    p.add_argument('--show', action='store_true', help='draw the simulation')
    p.add_argument('--seconds', type=float, default=300)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--noise', action='store_true', help='sim: pose noise, dropped frames, latency, failed grabs')
    p.add_argument('--field-physics', action='store_true',
                   help='sim: MIN_DUTY, spin stalls, walls and tag loss near edges as measured on the field')
    args = p.parse_args()
    if not args.config.is_file():
        p.error(f'Config not found: {args.config}. Pass --config with your actual calibrated JSON file.')
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
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
            params.update(sim.FIELD_PARAMS)
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
