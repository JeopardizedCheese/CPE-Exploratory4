"""Planner-side target lock: pick one stone and stay on it despite vision flicker.

Vision reports targets frame by frame; near thresholds they flicker. The planner
locks one target and keeps it until there is a real reason to drop it:
  - it has not been seen as a target for longer than max_missing_s
    (skipped while the robot itself is covering the stone: occluded=True)
  - a different known colour is seen at its position
  - the planner calls done() (placed) or release()

Targets are dicts in the vision packet format:
    {'color': 2, 'x': 68.8, 'y': 61.7, 'confidence': 0.55, 'approach_deg': -90.0}
x, y in arena mm. The colour chosen at lock time never changes.
"""
import math


class TargetLock:
    def __init__(self, max_missing_s=1.0, match_mm=30.0):
        self.max_missing_s = max_missing_s
        self.match_mm = match_mm
        self.target = None          # locked target dict (colour fixed at lock time)
        self.last_seen = 0.0
        self.reason = 'idle'

    def _near(self, a, b):
        return math.hypot(a['x'] - b['x'], a['y'] - b['y']) <= self.match_mm

    def update(self, targets, now, observations=(), occluded=False, choose=None):
        """targets: pickable targets this frame. observations: every detection
        (color, x, y), used to notice a colour change. Returns the locked target or None."""
        if self.target is not None:
            same = [t for t in targets if t['color'] == self.target['color'] and self._near(t, self.target)]
            if same:
                best = min(same, key=lambda t: math.hypot(t['x'] - self.target['x'], t['y'] - self.target['y']))
                color = self.target['color']
                self.target = dict(best, color=color)
                self.last_seen = now
                self.reason = 'tracking'
            elif any(o['color'] not in (0, self.target['color']) and self._near(o, self.target)
                     for o in list(targets) + list(observations)):
                self.release('colour changed')
            elif not occluded and now - self.last_seen > self.max_missing_s:
                self.release('lost')
            else:
                self.reason = 'occluded' if occluded else 'holding'
        if self.target is None and targets:
            pick = (choose or (lambda ts: max(ts, key=lambda t: t['confidence'])))(targets)
            if pick is not None:
                self.target = dict(pick)
                self.last_seen = now
                self.reason = 'locked'
        return self.target

    def release(self, reason='released'):
        self.target = None
        self.reason = reason

    def done(self):
        self.release('done')