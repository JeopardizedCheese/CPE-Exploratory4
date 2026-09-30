"""Measured jaw geometry and bounded stop/measure approach pulses."""
import math

DEFAULTS = {
    'grip_capture_back_mm': 35.0, 'grip_capture_front_mm': 12.0,
    'pickup_settle_s': .35, 'pickup_rest_mm_s': 40.0,
    'pickup_turn_clearance_mm': 80.0,
    'pickup_pulse_min_s': .04, 'pickup_pulse_max_s': .2,
    'pickup_speed_estimate_mm_s': 400.0,
}

def jaw_error(pose, target):
    """Error relative to the current jaws, never a frozen approach heading."""
    h = math.radians(pose.heading_deg)
    dx, dy = target['x']-pose.grip_x, target['y']-pose.grip_y
    return dx*math.cos(h)+dy*math.sin(h), -dx*math.sin(h)+dy*math.cos(h)

class Pickup:
    def __init__(self, options):
        self.o = dict(DEFAULTS, **options)
        for k in DEFAULTS:
            if isinstance(self.o[k], bool) or not isinstance(self.o[k], (int, float)) or not math.isfinite(self.o[k]) or self.o[k] <= 0:
                raise ValueError(k+' must be positive and finite')
        if not self.o['pickup_pulse_min_s'] <= self.o['pickup_pulse_max_s'] <= .2:
            raise ValueError('Pickup pulses require 0 < min <= max <= 0.2s')
        self.frames = []
        self.until = self.ready_at = 0.0
        self.command = (0.0, 0.0)
        self.start = None
        self.length = 0.0
        self.speed = self.o['pickup_speed_estimate_mm_s']

    @property
    def settle(self):
        return max(self.o['pickup_settle_s'], self.o.get('camera_delay_s', .2)+.1)

    def observe(self, p):
        if not self.frames or p.t > self.frames[-1].t:
            self.frames.append(p)
        self.frames = [q for q in self.frames if p.t-q.t <= .6]

    def at_rest(self, p):
        past = [q for q in self.frames if .10 <= p.t-q.t <= .5]
        if not past:
            return False
        q = past[0]
        dt = p.t-q.t
        turn = abs((p.heading_deg-q.heading_deg+180)%360-180)/dt
        return math.hypot(p.x-q.x, p.y-q.y)/dt <= self.o['pickup_rest_mm_s'] and turn <= 20

    def braking_distance(self, p):
        past = [q for q in self.frames if .15 <= p.t-q.t <= .3]
        speed = self.speed
        if past:
            q = past[0]
            speed = math.hypot(p.x-q.x, p.y-q.y)/(p.t-q.t)
        return max(self.o['pickup_turn_clearance_mm'], speed*(self.o.get('camera_delay_s', .2)+.3))

    def contains(self, p, target):
        along, side = jaw_error(p, target)
        return (-self.o['grip_capture_back_mm'] <= along <= self.o['grip_capture_front_mm']
                and abs(side) <= self.o['approach_max_side_mm'])

    def pause(self, now, restart=True):
        if not restart and not self.until:
            return  # a missed frame while already stopped adds no new coast
        if not restart and self.start is not None:
            self.length = max(.01, min(self.length, now-(self.until-self.length)))
        self.until = 0.0
        if restart:
            self.start = None
        self.ready_at = now+self.settle
        self.command = (0.0, 0.0)

    def waiting(self, now, p):
        """Command while moving/settling, or None when ready to decide again."""
        if self.until:
            if now < self.until:
                return self.command
            self.until = 0.0
            self.command = (0.0, 0.0)
            self.ready_at = now+self.settle
        if now < self.ready_at or not self.at_rest(p):
            return 0.0, 0.0
        if self.start is not None:
            distance = math.hypot(p.x-self.start.x, p.y-self.start.y)
            self.speed = max(40.0, min(1200.0, distance/self.length))
            self.start = None
        return None

    def pulse(self, now, p, direction, distance):
        self.length = max(self.o['pickup_pulse_min_s'], min(self.o['pickup_pulse_max_s'], distance/self.speed*.5))
        self.start, self.until = p, now+self.length
        v = direction*self.o['creep']
        self.command = (v, v)
        return self.command
