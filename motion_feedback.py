"""Camera feedback controllers. All angles are robot headings, clockwise positive.

Outputs are signed PWM duty fractions, not voltage or measured wheel speed.
No battery-specific minimum, start kick, or pulse-to-angle table is assumed.
"""
from collections import deque
from dataclasses import dataclass
import math


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def angle(v):
    return (v + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class Measurement:
    t: float
    x: float
    y: float
    heading: float
    speed: float       # forward speed at axle, mm/s; reverse is negative
    omega: float       # deg/s, clockwise positive


class VelocityEstimator:
    """Difference across a short time window, using axle position, not offset tag.

    t is frame receipt time. This cannot discover camera-internal buffering.
    """
    def __init__(self, axle_offset_mm=0, window_s=.15):
        self.axle_offset = axle_offset_mm
        self.window = window_s
        self.samples = deque()

    def update(self, pose):
        vals = (pose.t, pose.x, pose.y, pose.heading_deg)
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('Non-finite camera pose')
        h = math.radians(pose.heading_deg)
        x = pose.x - self.axle_offset * math.cos(h)
        y = pose.y - self.axle_offset * math.sin(h)
        if self.samples and pose.t <= self.samples[-1][0]:
            raise ValueError('Camera timestamps must increase')
        self.samples.append((pose.t, x, y, pose.heading_deg))
        while len(self.samples) > 2 and pose.t - self.samples[1][0] >= self.window:
            self.samples.popleft()
        t0, x0, y0, h0 = self.samples[0]
        dt = pose.t - t0
        if dt < self.window * .7:
            return None
        dh = angle(pose.heading_deg - h0)
        mid_h = math.radians(h0 + dh / 2)
        v = ((x - x0) * math.cos(mid_h) + (y - y0) * math.sin(mid_h)) / dt
        return Measurement(pose.t, x, y, pose.heading_deg, v, dh / dt)


@dataclass
class Settings:
    max_duty: float = .4
    base_duty: float = 0.0  # optional measured running duty; never a guessed 71% floor
    tolerance_deg: float = 1.0
    camera_delay_s: float = .2
    settle_s: float = .5
    turn_kp: float = .012
    turn_kd: float = .001
    speed_kp: float = .002
    speed_ki: float = .002
    acceleration_mm_s2: float = 80.0
    max_omega_deg_s: float = 90.0
    max_speed_mm_s: float = 300.0
    timeout_s: float = 20.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Settings must be finite')
        if not 0 < self.max_duty <= 1 or not 0 <= self.base_duty <= self.max_duty:
            raise ValueError('Require 0 <= base_duty <= max_duty <= 1, max_duty > 0')
        if not 0 < self.tolerance_deg < 15 or not 0 <= self.camera_delay_s <= 1:
            raise ValueError('Invalid angle tolerance or camera delay')
        if min(self.settle_s, self.turn_kp, self.speed_kp, self.speed_ki,
               self.acceleration_mm_s2, self.max_omega_deg_s, self.max_speed_mm_s,
               self.timeout_s) <= 0 or self.turn_kd < 0:
            raise ValueError('Gains, limits, and times must be positive (turn_kd may be zero)')


class FeedbackRun:
    """One bounded turn, speed trial, or power measurement; no route planning.

    A terminal fault stays latched. Caller must abort on lost pose/link and must
    explicitly start a new run; this class never resumes a fault by itself.
    """
    def __init__(self, mode, value, start, settings=None, duration=3, test_motion='forward'):
        self.s = settings or Settings()
        if mode not in ('turn', 'speed', 'duty') or not math.isfinite(value):
            raise ValueError('Invalid mode or target')
        if not math.isfinite(duration) or not 0 < duration <= 15:
            raise ValueError('Trial duration must be in (0, 15] seconds')
        if mode == 'turn' and not self.s.tolerance_deg < abs(value) <= 180:
            raise ValueError('Turn must be larger than tolerance and at most 180 degrees')
        if mode == 'speed' and not 0 < value < self.s.max_speed_mm_s:
            raise ValueError('Speed target must be positive and below the measured speed limit')
        if mode == 'duty' and not 0 < value <= self.s.max_duty:
            raise ValueError('Test duty must be positive and no greater than max_duty')
        if test_motion not in ('forward', 'left', 'right'):
            raise ValueError('Invalid test motion')
        self.mode, self.value, self.duration, self.test_motion = mode, value, duration, test_motion
        self.start = start
        self.target_heading = angle(start.heading + value) if mode == 'turn' else start.heading
        self.phase, self.reason = 'running', ''
        self.last_t, self.integral = start.t, 0.0
        self.still_since = None
        self.anchor = start
        self.last_command = (0.0, 0.0)
        self.debug = {}

    def abort(self, reason):
        if self.phase in ('done', 'aborted'):
            return (0.0, 0.0)  # preserve the first terminal result/reason
        self.phase, self.reason = 'aborted', reason
        self.last_command = (0.0, 0.0)
        return self.last_command

    def _settled(self, m):
        if abs(m.omega) <= 1.5 and abs(m.speed) <= 5:
            if self.still_since is None:
                self.still_since = m.t
            return m.t - self.still_since >= self.s.settle_s + self.s.camera_delay_s
        self.still_since = None
        return False

    def step(self, m):
        if self.phase in ('done', 'aborted'):
            return (0.0, 0.0)
        if not all(math.isfinite(v) for v in vars(m).values()):
            return self.abort('invalid_pose')
        dt = m.t - self.last_t
        if dt <= 0 or dt > .25:
            return self.abort('stale_pose')
        self.last_t = m.t
        if abs(m.omega) > self.s.max_omega_deg_s or abs(m.speed) > self.s.max_speed_mm_s:
            return self.abort('measured_speed_limit')
        elapsed = m.t - self.start.t
        if elapsed > self.s.timeout_s:
            return self.abort('timeout_target_not_achieved')
        err = angle(self.target_heading - m.heading)
        if self.mode == 'turn' and abs(abs(err) - 180) < 1e-8:
            err = math.copysign(180, self.value)  # honor the requested sign at the half-turn tie
        predicted_err = err - m.omega * self.s.camera_delay_s
        self.debug = dict(error_deg=err, predicted_error_deg=predicted_err,
                          measured_mm_s=m.speed, measured_deg_s=m.omega)
        l = r = 0.0
        if self.mode == 'turn':
            if abs(err) <= self.s.tolerance_deg:
                self.phase = 'settling'
                if self._settled(m):
                    self.phase, self.reason = 'done', 'within_requested_camera_tolerance'
            else:
                self.phase, self.still_since = 'running', None
                direction = math.copysign(1.0, err)
                # Coast early when the delayed view predicts that we have reached the target.
                remaining = direction * predicted_err
                if remaining > self.s.tolerance_deg * .5:
                    effort = self.s.base_duty + self.s.turn_kp * remaining
                    effort -= self.s.turn_kd * max(0, direction * m.omega)
                    l = direction * clip(effort, 0, self.s.max_duty)
                    r = -l
        elif elapsed >= self.duration:
            self.phase = 'settling'
            if self._settled(m):
                self.phase, self.reason = 'done', 'trial_complete_check_measured_results'
        elif self.mode == 'duty':
            l, r = {'forward': (self.value, self.value), 'left': (-self.value, self.value),
                    'right': (self.value, -self.value)}[self.test_motion]
        else:
            if abs(err) > 20:
                return self.abort('heading_deviation')
            target = min(self.value, self.s.acceleration_mm_s2 * elapsed)
            speed_error = target - m.speed
            candidate = self.integral + self.s.speed_ki * speed_error * dt
            effort = self.s.base_duty + self.s.speed_kp * speed_error + candidate
            # Integrate only when not saturating farther into the limit.
            if (0 <= effort <= self.s.max_duty or effort > self.s.max_duty and speed_error < 0
                    or effort < 0 and speed_error > 0):
                self.integral = clip(candidate, -self.s.max_duty, self.s.max_duty)
            effort = clip(self.s.base_duty + self.s.speed_kp * speed_error + self.integral,
                          0, self.s.max_duty)
            steer = clip(self.s.turn_kp * err - self.s.turn_kd * m.omega,
                         -min(effort, self.s.max_duty - effort), min(effort, self.s.max_duty - effort))
            l, r = effort + steer, effort - steer
            self.debug.update(target_mm_s=target, speed_error_mm_s=speed_error,
                              saturated=effort >= self.s.max_duty - .001)
        moved = (abs(angle(m.heading - self.anchor.heading)) >= .5 if self.mode == 'turn'
                 else math.hypot(m.x - self.anchor.x, m.y - self.anchor.y) >= 3)
        if moved or max(abs(l), abs(r)) < .01 or self.mode == 'duty':
            self.anchor = m
        elif m.t - self.anchor.t > 4:
            return self.abort('no_measured_progress_at_requested_power')
        self.last_command = (l, r)
        return self.last_command
