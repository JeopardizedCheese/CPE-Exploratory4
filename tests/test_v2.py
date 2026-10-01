"""V1 / V2 split (profiles.py): V1 = the field-tested behaviour, V2 = pile fix + outermost
pile stone + target commitment. Vision parts are in tests/test_pile.py."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import autonomy
import profiles
from autonomy import Planner
from target_lock import TargetLock
from tests.test_autonomy import config, pose

ROOT = Path(__file__).resolve().parent.parent


def t(color, x, y, conf=.6, approach=None):
    d = {'color': color, 'x': x, 'y': y, 'confidence': conf}
    if approach is not None:
        d['approach_deg'] = approach
    return d


class CommittedLockTests(unittest.TestCase):
    """track_observations: a locked stone stays locked while it is still seen at its spot,
    even when vision no longer offers it as a pickable target (the robot closing in blocks
    its corridor, the pile shifts)."""

    def test_follows_the_stone_when_it_is_no_longer_a_target(self):
        legacy, commit = TargetLock(), TargetLock(track_observations=True)
        for lock in (legacy, commit):
            lock.update([t(1, 100, 100, approach=0)], 0.0)
        seen = [{'color': 1, 'x': 104, 'y': 101}]
        for i in range(1, 21):
            legacy.update([], 0.1 * i, observations=seen)
            got = commit.update([], 0.1 * i, observations=seen)
        self.assertEqual(legacy.reason, 'lost')
        self.assertEqual((got['x'], got['y'], got['approach_deg']), (104, 101, 0))
        self.assertEqual(commit.reason, 'seen')

    def test_neighbour_of_another_colour_does_not_break_it(self):
        legacy, commit = TargetLock(), TargetLock(track_observations=True)
        for lock in (legacy, commit):
            lock.update([t(1, 100, 100)], 0.0)
        pile = [{'color': 1, 'x': 100, 'y': 100}, {'color': 3, 'x': 130, 'y': 100}]
        legacy.update([], 0.1, observations=pile)
        self.assertEqual(legacy.reason, 'colour changed')
        self.assertIsNotNone(commit.update([], 0.1, observations=pile))

    def test_still_releases_on_a_real_colour_change(self):
        lock = TargetLock(track_observations=True)
        lock.update([t(1, 100, 100)], 0.0)
        lock.update([], 0.1, observations=[{'color': 3, 'x': 101, 'y': 100}])
        self.assertEqual(lock.reason, 'colour changed')

    def test_still_lost_when_the_stone_is_gone(self):
        lock = TargetLock(track_observations=True)
        lock.update([t(1, 100, 100)], 0.0)
        self.assertIsNone(lock.update([], 1.2, observations=[{'color': 0, 'x': 100, 'y': 100}]))
        self.assertEqual(lock.reason, 'lost')


def planner(**autonomy_options):
    cfg = config()
    cfg['autonomy'] = autonomy_options
    return Planner(cfg)


class CommittedPlannerTests(unittest.TestCase):
    def drive_to_stage(self, p, seconds):
        """Lock a stone, then vision only reports it as an observation (not pickable)."""
        stone = t(2, 1000, 600, approach=0)
        p.step(0.0, pose(500, 600, 0, 0.0), [stone], [stone])
        self.assertEqual(p.state, 'GOTO_STAGE')
        seen = [{'color': 2, 'x': 1000, 'y': 600}]
        for i in range(1, int(seconds * 10) + 1):
            now = 0.1 * i
            p.step(now, pose(500, 600, 0, now), [], seen)
        return p

    def test_v2_keeps_going_to_a_stone_that_is_still_there(self):
        self.assertEqual(self.drive_to_stage(planner(commit_target=True), 2.0).state, 'GOTO_STAGE')

    def test_v1_gives_up_after_a_second(self):
        self.assertNotEqual(self.drive_to_stage(planner(), 2.0).state, 'GOTO_STAGE')

    def first_lock_time(self, p, targets):
        p.skip = [(1000, 600, 25.0)]                     # skipped at t=0 for skip_s (25 s)
        for i in range(0, 200):                          # 20 s: V1's skip lasts 25
            now = 0.1 * i
            p.step(now, pose(500, 600, 0, now), targets, targets)
            if p.state == 'GOTO_STAGE':
                return now
        return None

    def test_only_stone_retried_after_skip_alone_s(self):
        only = [t(2, 1000, 600)]
        self.assertIsNone(self.first_lock_time(planner(), only))              # V1: parks for 25 s
        locked = self.first_lock_time(planner(skip_alone_s=6), only)
        self.assertIsNotNone(locked)
        self.assertTrue(6.0 <= locked < 8.0, locked)

    def test_skip_alone_does_not_pick_a_skipped_stone_over_another(self):
        both = [t(2, 1000, 600), t(3, 900, 300)]
        p = planner(skip_alone_s=6)
        self.first_lock_time(p, both)
        self.assertEqual(p.lock.target['color'], 3)


class ProfileTests(unittest.TestCase):
    def test_v1_sets_every_switch_to_its_legacy_value(self):
        cfg = config()
        profiles.apply_profile(cfg, 'v1')
        self.assertEqual(cfg['profile'], 'v1')
        self.assertEqual(cfg['vision']['pile_edge_pixels'], 'legacy')
        self.assertFalse(cfg['vision']['pile_outermost'])
        self.assertFalse(cfg['vision']['pile_regions'])
        self.assertFalse(cfg['autonomy']['commit_target'])
        self.assertIsNone(cfg['autonomy']['skip_alone_s'])
        self.assertTrue(cfg['vision']['pile_mode'])                      # untouched calib values stay
        self.assertEqual(Planner(cfg).o, Planner(config()).o)            # V1 = the code defaults

    def test_v2_values(self):
        cfg = config()
        profiles.apply_profile(cfg, 'v2')
        self.assertEqual(cfg['profile'], 'v2')
        self.assertEqual(cfg['vision']['pile_edge_pixels'], 'nearest')
        self.assertTrue(cfg['vision']['pile_outermost'])
        self.assertTrue(cfg['vision']['pile_regions'])
        self.assertTrue(cfg['autonomy']['commit_target'])
        self.assertTrue(Planner(cfg).lock.track_observations)

    def test_set_wins_over_the_profile(self):
        cfg = config()
        autonomy.prepare_config(cfg, 'v2', ['vision.pile_outermost=false', 'commit_target=false'])
        self.assertFalse(cfg['vision']['pile_outermost'])
        self.assertFalse(cfg['autonomy']['commit_target'])
        self.assertEqual(cfg['vision']['pile_edge_pixels'], 'nearest')

    def test_profile_does_not_write_the_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'calib.json'
            path.write_text(json.dumps(config()))
            before = path.read_text()
            cfg = json.loads(before)
            autonomy.prepare_config(cfg, 'v2', [])
            self.assertEqual(path.read_text(), before)

    def test_vision_switches_by_name(self):
        cfg = {}
        autonomy.apply_overrides(cfg, ['vision.pile_edge_pixels=legacy', 'vision.own_reach_mm=12',
                                       'vision.pile_outermost=true', 'vision.pile_regions=false'])
        self.assertEqual(cfg['vision'], {'pile_edge_pixels': 'legacy', 'own_reach_mm': 12,
                                         'pile_outermost': True, 'pile_regions': False})
        for bad in ('vision.pile_edge_pixel=nearest',      # typo
                    'vision.pile_edge_pixels=closest',     # bad value
                    'vision.own_reach_mm=-3', 'vision.own_reach_mm=wide',
                    'vision.pile_outermost=yes',
                    'vision.background_delta=20'):         # not a named switch
            with self.assertRaises(ValueError, msg=bad):
                autonomy.apply_overrides({}, [bad])

    def test_unknown_profile_rejected(self):
        with self.assertRaises(ValueError):
            profiles.apply_profile({}, 'v3')

    def test_autonomy2_runs_the_v2_profile(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'calib.json'
            path.write_text(json.dumps(config()))
            for script, name in (('autonomy.py', 'V1'), ('autonomy2.py', 'V2')):
                out = subprocess.run([sys.executable, str(ROOT / script), '--check-config', '--config', str(path)],
                                     capture_output=True, text=True, cwd=ROOT)
                self.assertIn(f'Version: {name}', out.stdout + out.stderr)


if __name__ == '__main__':
    unittest.main()
