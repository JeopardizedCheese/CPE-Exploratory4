"""Return a visible robot from a field edge using measured, bounded movements.

The calibrated rectangle is a navigation boundary, not a camera crop. Coordinates
outside it are allowed for recovery. Every movement still needs a fresh tag pose.
"""
import math
from dataclasses import replace


DEFAULTS = {
    'wall_recovery_enabled': True,
    'wall_margin_mm': 200.0,
    'wall_body_margin_mm': 20.0,
    'wall_resume_mm': 60.0,
    'wall_recovery_speed': .18,   # legacy drive command; obeys the configured min_duty
    'wall_recovery_turn': .5,     # = pulse_power; at .25 (duty ~.74) a spin often never starts
    'wall_pulse_s': .15,          # drive pulse; .10 s barely moved the robot
    'wall_turn_pulse_s': .2,      # field: .07-.1 s turn pulses barely turned, .2 s ~30 deg
    'wall_path_mm': 120.0,        # stone check ahead of a drive pulse: .15 s at ~400 mm/s + coast
    'wall_settle_s': .35,
    'wall_rest_mm_s': 80.0,       # before each pulse the camera must show the robot this slow
    'wall_timeout_s': 12.0,
    'wall_no_progress_s': 3.0,
    'wall_retry_s': 3.0,          # WALL_BLOCKED stays stopped this long, then tries again
    'wall_max_outside_mm': 200.0,
    'wall_pose_max_age_s': .25,
}


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WallGuard:
    def __init__(self, cfg, options=None):
        self.o = dict(DEFAULTS, **(options or {}))
        for key in DEFAULTS:
            if key == 'wall_recovery_enabled':
                if not isinstance(self.o[key], bool):
                    raise ValueError(key + ' must be true or false')
            elif isinstance(self.o[key], bool) or not isinstance(self.o[key], (int, float)) or not math.isfinite(self.o[key]) or self.o[key] <= 0:
                raise ValueError(key + ' must be positive and finite')
        if max(self.o['wall_recovery_speed'], self.o['wall_recovery_turn']) > 1:
            raise ValueError('Wall motor commands must be <= 1')
        if self.o['wall_pulse_s'] > .2 or self.o['wall_turn_pulse_s'] > .4 or self.o['wall_pose_max_age_s'] > .3:
            raise ValueError('Wall pulses must be <= 0.2s (drive) / 0.4s (turn) and pose age <= 0.3s')
        self.w, self.h = map(float, cfg['arena']['size_mm'])
        tag = cfg.get('robot_tag', {})
        self.axle = float(tag.get('axle_offset_mm', 0))
        self.fp = dict(tag.get('footprint_mm', {'front': 170, 'back': 110, 'left': 105, 'right': 105}))
        if any(not math.isfinite(v) or v <= 0 for v in self.fp.values()):
            raise ValueError('Robot footprint dimensions must be positive and finite')
        self.zones = cfg.get('zones', {})
        self.enabled = self.o['wall_recovery_enabled']
        if self.enabled and 2 * (self.o['wall_margin_mm'] + self.o['wall_resume_mm']) >= min(self.w, self.h):
            raise ValueError('Field is too small for wall_margin_mm + wall_resume_mm; measure smaller margins')
        self.active = self.failed = self.completed = False
        self.phase, self.reason = 'idle', ''
        self.started = self.progress_at = self.until = 0.0
        self.anchor = None
        self.best_clearance = None
        self.pulses = 0
        self.failed_at = 0.0
        self.command = (0.0, 0.0)
        self.last_pose_t = None
        self.frame = self.prev_frame = None
        self.settle_s = max(self.o['wall_settle_s'], float(self.o.get('camera_delay_s', .2)) + .1)

    def corners(self, p):
        h = math.radians(p.heading_deg)
        c, s = math.cos(h), math.sin(h)
        return [(p.x + f*c - r*s, p.y + f*s + r*c)
                for f in (self.fp['front'], -self.fp['back'])
                for r in (-self.fp['left'], self.fp['right'])]

    def clearances(self, p, extra=0.0):
        pts = self.corners(p)
        margin = self.o['wall_margin_mm'] + extra
        body = self.o['wall_body_margin_mm'] + extra
        return (min(p.x-margin, min(x for x, _ in pts)-body),
                min(self.w-p.x-margin, self.w-max(x for x, _ in pts)-body),
                min(p.y-margin, min(y for _, y in pts)-body),
                min(self.h-p.y-margin, self.h-max(y for _, y in pts)-body))

    def _body_room(self, p):
        """Distance from the nearest body corner to the field edge (negative: past it)."""
        pts = self.corners(p)
        return min(min(x for x, _ in pts), self.w-max(x for x, _ in pts),
                   min(y for _, y in pts), self.h-max(y for _, y in pts))

    def _tag_room(self, p):
        return min(p.x, p.y, self.w-p.x, self.h-p.y)

    def safe_pose(self, p):
        return not self.enabled or min(self.clearances(p)) >= 0

    def clamp_tag_goal(self, x, y):
        if not self.enabled:
            return x, y
        radius = max(math.hypot(f, r) for f in (self.fp['front'], self.fp['back'])
                     for r in (self.fp['left'], self.fp['right']))
        margin = max(self.o['wall_margin_mm'], radius + self.o['wall_body_margin_mm'])
        return max(margin, min(self.w-margin, x)), max(margin, min(self.h-margin, y))

    def _translated(self, p, distance):
        h = math.radians(p.heading_deg)
        dx, dy = distance*math.cos(h), distance*math.sin(h)
        return replace(p, x=p.x+dx, y=p.y+dy, grip_x=p.grip_x+dx, grip_y=p.grip_y+dy)

    def _rotated(self, p, degrees):
        h, d = math.radians(p.heading_deg), math.radians(degrees)
        ax, ay = p.x-self.axle*math.cos(h), p.y-self.axle*math.sin(h)
        x, y = ax+self.axle*math.cos(h+d), ay+self.axle*math.sin(h+d)
        gx, gy = p.grip_x-p.x, p.grip_y-p.y
        return replace(p, x=x, y=y, heading_deg=p.heading_deg+degrees,
                       grip_x=x+gx*math.cos(d)-gy*math.sin(d),
                       grip_y=y+gx*math.sin(d)+gy*math.cos(d))

    def _contains(self, p, ob, margin=25):
        h = math.radians(p.heading_deg)
        dx, dy = ob['x']-p.x, ob['y']-p.y
        f, r = dx*math.cos(h)+dy*math.sin(h), -dx*math.sin(h)+dy*math.cos(h)
        return (-self.fp['back']-margin <= f <= self.fp['front']+margin
                and -self.fp['left']-margin <= r <= self.fp['right']+margin)

    def _path_clear(self, poses, observations, carrying):
        initial = poses[0]
        # The held stone is masked by perception. Do not drive over observed stones
        # or unknown objects, including objects already touching the robot's edge.
        for ob in observations:
            # An object already inside the padding may be escaped by moving away.
            # Requiring an empty padding ring at the start traps a robot beside a pile.
            def separation(p):
                h = math.radians(p.heading_deg)
                dx, dy = ob['x']-p.x, ob['y']-p.y
                f, r = dx*math.cos(h)+dy*math.sin(h), -dx*math.sin(h)+dy*math.cos(h)
                df = max(-self.fp['back']-f, f-self.fp['front'])
                dr = max(-self.fp['left']-r, r-self.fp['right'])
                return math.hypot(max(df, 0), max(dr, 0)) + min(max(df, dr), 0)
            d0 = separation(initial)
            touching_padding = self._contains(initial, ob)
            for p in poses[1:]:
                if self._contains(p, ob):
                    if not touching_padding or separation(p) < d0-1:
                        return False
        if carrying is not None:
            for key, zone in self.zones.items():
                if int(str(key).split('_')[0]) == carrying:
                    continue
                x, y = zone['center_mm']
                d0 = math.hypot(initial.grip_x-x, initial.grip_y-y)
                for p in poses[1:]:
                    d = math.hypot(p.grip_x-x, p.grip_y-y)
                    if d < zone['radius_mm']+20 and d < d0-1:
                        return False
        return True

    def _escape(self, p, observations, carrying):
        extra = self.o['wall_resume_mm']
        deficits = [max(0, -v) for v in self.clearances(p, extra)]
        options = []
        for direction in (1, -1):
            path = [self._translated(p, direction*k*10) for k in range(int(self.o['wall_path_mm'])//10+1)]
            after = [max(0, -v) for v in self.clearances(path[-1], extra)]
            gain = sum(deficits)-sum(after)
            if (gain > 1 and all(b <= a+.01 for a, b in zip(deficits, after))
                    and self._path_clear(path, observations, carrying)):
                options.append((gain, direction))
        if options:
            direction = max(options)[1]
            v = direction*self.o['wall_recovery_speed']
            return (v, v), 'reverse_inward' if direction < 0 else 'forward_inward', self.o['wall_pulse_s']
        # A tangent heading needs a turn: toward facing inward, or facing the wall
        # to reverse out. Do not grind a corner into a wall to try to free it: the
        # body must keep its room over the whole arc the pulse may sweep.
        h = math.radians(p.heading_deg)
        left, right, top, bottom = deficits
        wanted = math.atan2(top-bottom, left-right)
        errors = (wrap(wanted-h), wrap(wanted+math.pi-h))  # may enter in reverse
        if min(abs(e) for e in errors) < math.radians(3):
            return None  # translation was blocked by an obstacle
        room = min(self.o['wall_body_margin_mm'], self._body_room(p)) - 1
        tag_room = self._tag_room(p)
        best = None
        for err in errors:
            sign = math.copysign(1, err)
            # One pulse turns ~3x more or less than planned: check past the goal heading.
            sweep = min(360, max(abs(math.degrees(err)) + 30, 100))
            path = [self._rotated(p, sign*d) for d in range(0, int(sweep)+1, 5)]
            if any(self._body_room(q) < room for q in path) or not self._path_clear(path, observations, carrying):
                continue
            # Prefer a turn that keeps the tag off the edge, where the camera loses it.
            key = (min(self._tag_room(q) for q in path) < tag_room-10, abs(err))
            if best is None or key < best[0]:
                best = (key, sign)
        if best is None:
            return None
        v = best[1]*self.o['wall_recovery_turn']
        return (v, -v), 'turn_inward', self.o['wall_turn_pulse_s']

    def _at_rest(self):
        """The last two camera frames show the robot (nearly) stopped. With the wheels
        off it only slows down, so the mean speed over the gap bounds its speed now."""
        a, b = self.prev_frame, self.frame
        if a is None or not 0 < b.t-a.t <= .5:
            return False
        dt = b.t-a.t
        turned = abs(math.degrees(wrap(math.radians(b.heading_deg-a.heading_deg))))
        return math.hypot(b.x-a.x, b.y-a.y)/dt <= self.o['wall_rest_mm_s'] and turned/dt <= 30

    def pause(self, now, reason):
        self.command = (0.0, 0.0)
        if self.active and not self.failed:
            self.phase, self.reason = 'settle', reason
            self.until = now+self.settle_s

    def _fail(self, now, reason):
        self.active, self.failed, self.phase, self.reason = True, True, 'blocked', reason
        self.failed_at = now
        self.command = (0.0, 0.0)
        return self.command

    def step(self, now, pose, observations=(), carrying=None, forecast=None):
        """None lets the planner continue; a pair overrides all mission motion."""
        self.completed = False
        if not self.enabled:
            return None
        if pose is None or not 0 <= now-pose.t <= self.o['wall_pose_max_age_s']:
            self.pause(now, 'wall_wait_fresh_tag')
            return (0.0, 0.0) if self.active else None
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.heading_deg, pose.grip_x, pose.grip_y, pose.t)):
            return self._fail(now, 'wall_invalid_pose')
        new_frame = pose.t != self.last_pose_t
        if new_frame:
            self.last_pose_t = pose.t
            self.prev_frame, self.frame = self.frame, pose
        if self.failed:
            if now-self.failed_at < self.o['wall_retry_s']:
                return (0.0, 0.0)
            # Try again: the robot may have been moved, or a stone in the way pushed aside.
            self.active = self.failed = False
            self.phase = 'idle'
        if not self.active:
            if self.safe_pose(pose) and (forecast is None or self.safe_pose(forecast)):
                return None
            self.active = True
            self.started = self.progress_at = now
            self.anchor = pose
            self.pulses = 0
            self.best_clearance = min(self.clearances(pose))
            self.pause(now, 'wall_stop_and_measure')
        if min(pose.x, pose.y, self.w-pose.x, self.h-pose.y) < -self.o['wall_max_outside_mm']:
            return self._fail(now, 'wall_outside_recovery_range')
        if now-self.started > self.o['wall_timeout_s']:
            return self._fail(now, 'wall_recovery_timeout')
        if new_frame and self.anchor is not None:
            clearance = min(self.clearances(pose))
            if not self.pulses:
                # Still stopping: the camera shows the motion from before the stop ~0.2 s
                # late, and the wheels coast. Judge only motion after our own first pulse.
                self.best_clearance = clearance
            elif clearance < self.best_clearance-25:
                # Compare to the best measured clearance of this attempt. Comparing
                # only adjacent frames misses a slow, cumulative drive into the wall.
                return self._fail(now, 'wall_motion_went_outward')
            self.best_clearance = max(self.best_clearance, clearance)
            moved = math.hypot(pose.x-self.anchor.x, pose.y-self.anchor.y)
            turned = abs(math.degrees(wrap(math.radians(pose.heading_deg-self.anchor.heading_deg))))
            if moved >= 8 or turned >= 3:
                self.progress_at, self.anchor = now, pose
        if self.phase == 'pulse':
            if now < self.until:
                return self.command
            self.pause(now, 'wall_measure_after_pulse')
        if now < self.until or not new_frame:
            return (0.0, 0.0)
        if min(self.clearances(pose, self.o['wall_resume_mm'])) >= 0:
            self.active, self.completed, self.phase, self.reason = False, True, 'complete', 'wall_back_inside'
            return (0.0, 0.0)
        if now-self.progress_at > self.o['wall_no_progress_s']:
            return self._fail(now, 'wall_no_progress')
        if not self._at_rest():
            self.reason = 'wall_wait_until_stopped'   # decide from where the robot came to rest
            return (0.0, 0.0)
        escape = self._escape(pose, observations, carrying)
        if escape is None:
            return self._fail(now, 'wall_no_clear_escape')
        self.command, self.reason, seconds = escape
        self.pulses += 1
        self.phase, self.until = 'pulse', now+seconds
        return self.command

    def diagnostics(self, pose=None):
        return {'wall_phase': self.phase, 'wall_reason': self.reason,
                'wall_clearance_mm': min(self.clearances(pose)) if pose else None,
                'wall_command_until': self.until if self.phase == 'pulse' else None}
