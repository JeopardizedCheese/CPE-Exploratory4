"""Check fixed scoring-circle positions before trusting an empty-field reference.

This detects changes; it does not realign frames or repair localization. Occluded
or missing circles produce 'unverified', never proof that calibration is valid.
"""
import time

import numpy as np
from find_zones import find_zones


class ReferenceGuard:
    def __init__(self, cfg, background):
        self.cfg = cfg
        self.options = cfg.get('vision', {}).get('reference_guard', {})
        self.enabled = self.options.get('enabled', False)
        self.scale = float(cfg.get('arena', {}).get('mm_per_px', 2))
        self.reference = []
        self.bad_since = None
        if self.enabled and background is not None:
            self.reference, _ = find_zones(background, cfg, classify_colors=False)

    def check(self, frame, now=None):
        """A 'moved'/'unverified' verdict is reported only once it has lasted
        reference_guard.hold_s (default 0: at once). Until then status stays 'ok'
        and 'pending' carries the raw verdict, so a hand or one noisy frame does
        not stop the robot."""
        result = self._check_once(frame)
        hold = float(self.options.get('hold_s', 0))
        if result['status'] not in ('moved', 'unverified'):
            self.bad_since = None
            return result
        now = time.monotonic() if now is None else now
        if self.bad_since is None:
            self.bad_since = now
        if now - self.bad_since < hold:
            result['pending'], result['status'] = result['status'], 'ok'
        return result

    def _check_once(self, frame):
        result = {'status': 'disabled', 'reference_markers': len(self.reference), 'matched': 0, 'shifts': []}
        if not self.enabled:
            return result
        result['status'] = 'unverified'
        minimum = int(self.options.get('min_markers', 3))
        if len(self.reference) < minimum:
            return result
        current, _ = find_zones(frame, self.cfg, classify_colors=False)
        pairs = []
        for i, ref in enumerate(self.reference):
            for j, now in enumerate(current):
                ratio = now['radius_px'] / ref['radius_px']
                distance = float(np.linalg.norm(np.array(ref['center_px']) - now['center_px'])) * self.scale
                if .7 <= ratio <= 1.3 and distance <= self.options.get('max_match_mm', 250):
                    pairs.append((distance, i, j))
        used_ref, used_now = set(), set()
        for distance, i, j in sorted(pairs):
            if i in used_ref or j in used_now:
                continue
            used_ref.add(i)
            used_now.add(j)
            result['shifts'].append({'reference_px': self.reference[i]['center_px'],
                                     'current_px': current[j]['center_px'],
                                     'distance_px': distance / self.scale, 'distance_mm': distance})
        result['matched'] = len(used_ref)
        if len(used_ref) < minimum:
            return result
        distances = [s['distance_mm'] for s in result['shifts']]
        result['median_shift_mm'] = float(np.median(distances))
        result['max_shift_mm'] = float(max(distances))
        shifted = sum(d > self.options.get('max_shift_mm', 8) for d in distances)
        result['status'] = 'moved' if shifted >= self.options.get('min_shifted_markers', 2) else 'ok'
        return result
