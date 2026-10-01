"""V3: grip check by the gripper camera (HuskyLens on the ESP32, "look" command)."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import autonomy
import profiles
import sim
from autonomy import Planner, grip_verdict, grip_check_problem, wait_for_gripcam
from tests.test_autonomy import config, pose

ROOT = Path(__file__).resolve().parents[1]
IDS = {'empty': [1, 2], 'single': [3, 4], 'multiple': [5, 6]}
STONE = {'color': 2, 'x': 1120.0, 'y': 600.0, 'confidence': 1}


class Verdict(unittest.TestCase):
    def test_empty_needs_all_five(self):
        self.assertEqual(grip_verdict([1, 2, 1, 1, 2], IDS), 'empty')
        self.assertEqual(grip_verdict([1, 1, 1, 1, 3], IDS), 'unsure')
        self.assertEqual(grip_verdict([1, 1, 1, 1], IDS), 'unsure')        # only 4 readings came

    def test_held_needs_four(self):
        self.assertEqual(grip_verdict([3, 4, 3, 3, 1], IDS), 'single')
        self.assertEqual(grip_verdict([5, 6, 5, 5, 0], IDS), 'multiple')
        self.assertEqual(grip_verdict([3, 3, 3, 5, 5], IDS), 'unsure')     # single vs multiple split

    def test_nothing_recognised_is_unsure(self):
        self.assertEqual(grip_verdict([0, 0, 0, 0, 0], IDS), 'unsure')
        self.assertEqual(grip_verdict([], IDS), 'unsure')
        self.assertEqual(grip_verdict([7, 7, 7, 7, 7], IDS), 'unsure')     # an ID not in the map


def gripping(grip_check=True):
    cfg = config()
    cfg['autonomy'] = {'grip_check': grip_check}
    p = Planner(cfg)
    p.lock.target, p.lock.last_seen = dict(STONE), 0.0
    p._start_grip(dict(STONE), 0.0, [])
    return p


def status(look=None, servo=40):
    st = {'state': 'RUNNING', 'servo': [servo], 'gripcam': 'ok'}
    if look is not None:
        st['look'] = look
    return st


def step(p, t, st):
    return p.step(t, pose(1000, 600, 0, t), [], [STONE], st)


class GripState(unittest.TestCase):
    def ask(self):
        p = gripping()
        _, _, ev = step(p, 0.1, status(servo=10))                 # jaws still closing
        self.assertNotIn('look', [c for c, _ in ev])
        _, _, ev = step(p, 0.2, status())                          # closed: ask once
        looks = [f for c, f in ev if c == 'look']
        self.assertEqual(len(looks), 1)
        _, _, ev = step(p, 0.3, status())
        self.assertEqual(p.state, 'GRIP')
        self.assertNotIn('look', [c for c, _ in ev])               # asked once, now waiting
        return p, looks[0]['n']

    def answer(self, ids, t=0.6):
        p, n = self.ask()
        _, _, ev = step(p, t, status({'n': n, 'done': True, 'ids': ids}))
        return p, ev

    def test_empty_opens_backs_off_and_skips_the_spot(self):
        p, ev = self.answer([1, 1, 1, 1, 1])
        self.assertEqual(p.state, 'BACKOFF')
        self.assertIn(('grip', {'p': 'open'}), ev)
        self.assertIsNone(p.carrying)
        self.assertIsNone(p.lock.target)
        self.assertEqual(p.skip[-1][:2], (STONE['x'], STONE['y']))
        self.assertEqual(p.debug['grip_verdict'], 'empty')
        self.assertEqual(p.debug['grip_check_ids'], [1, 1, 1, 1, 1])

    def test_single_and_multiple_carry_without_the_pick_check(self):
        for ids, verdict in (([3, 3, 4, 3, 1], 'single'), ([5, 5, 5, 6, 3], 'multiple')):
            p, ev = self.answer(ids)
            self.assertEqual(p.state, 'CARRY', verdict)
            self.assertTrue(p.pick_checked)
            self.assertEqual(p.carrying, 2)
            self.assertNotIn(('grip', {'p': 'open'}), ev)
            self.assertIn(verdict, p.events_log[-1][3])

    def test_unsure_carries_like_v2(self):
        p, _ = self.answer([1, 1, 1, 1, 3])                        # 4 of 5 is not enough for Empty
        self.assertEqual(p.state, 'CARRY')
        self.assertFalse(p.pick_checked)                           # the overhead pick check still runs

    def test_an_old_look_is_not_this_grip_check(self):
        p, n = self.ask()
        step(p, 0.35, status({'n': n - 1, 'done': True, 'ids': [1, 1, 1, 1, 1]}))
        self.assertEqual(p.state, 'GRIP')
        step(p, 0.38, status({'n': n, 'done': False, 'ids': [1, 1]}))   # not finished yet
        self.assertEqual(p.state, 'GRIP')

    def test_lost_look_packet_is_sent_once_more(self):
        p, n = self.ask()
        _, _, ev = step(p, 0.7, status())                          # 0.5 s: the firmware never saw it
        self.assertEqual([f for c, f in ev if c == 'look'], [{'n': n}])
        _, _, ev = step(p, 0.9, status())
        self.assertNotIn('look', [c for c, _ in ev])

    def test_no_answer_is_unsure(self):
        p, _ = self.ask()
        step(p, 1.8, status())                                     # > grip_check_timeout_s
        self.assertEqual(p.state, 'CARRY')
        self.assertFalse(p.pick_checked)
        self.assertEqual(p.debug['grip_verdict'], 'unsure')

    def test_every_grip_asks_again(self):
        p, ev = self.answer([3, 3, 3, 3, 3])
        p._start_grip(dict(STONE), 2.0, [])
        _, _, ev = step(p, 2.1, status())
        self.assertEqual(len([c for c, _ in ev if c == 'look']), 1)

    def test_switch_off_never_asks(self):
        p = gripping(grip_check=False)
        _, _, ev = step(p, 0.2, status())
        self.assertEqual(p.state, 'CARRY')
        self.assertNotIn('look', [c for c, _ in ev])


class Profiles(unittest.TestCase):
    def test_v3_is_v2_plus_grip_check(self):
        v2, v3 = profiles.apply_profile(config(), 'v2'), profiles.apply_profile(config(), 'v3')
        self.assertTrue(v3['autonomy'].pop('grip_check'))
        self.assertFalse(v2['autonomy'].pop('grip_check'))
        v2.pop('profile'), v3.pop('profile')
        self.assertEqual(v2, v3)

    def test_v1_and_v2_switch_it_off_explicitly(self):
        for name in ('v1', 'v2'):
            cfg = config()
            cfg['autonomy'] = {'grip_check': True}                 # even if calib.json had it on
            self.assertFalse(Planner(profiles.apply_profile(cfg, name)).o['grip_check'])

    def test_set_turns_it_off(self):
        cfg = autonomy.prepare_config(config(), 'v3', ['grip_check=false'])
        self.assertFalse(Planner(cfg).o['grip_check'])

    def test_ids_come_from_the_config_not_the_profile(self):
        cfg = config()
        cfg['autonomy'] = {'grip_check_ids': {'empty': [1], 'single': [2, 3], 'multiple': [4]}}
        autonomy.prepare_config(cfg, 'v3', [])
        self.assertEqual(Planner(cfg).o['grip_check_ids']['single'], [2, 3])

    def test_config_problems(self):
        self.assertIsNone(grip_check_problem(config()))
        bad = [{'grip_check': 'yes'},
               {'grip_check_ids': {'empty': [1], 'held': [2]}},
               {'grip_check_ids': {'empty': [1], 'single': [1]}},
               {'grip_check_ids': {'empty': ['1']}},
               {'grip_check_ids': {'empty': [0]}},
               {'grip_check_votes': 6}, {'grip_check_empty_votes': 0}]
        for autonomy_options in bad:
            cfg = config()
            cfg['autonomy'] = autonomy_options
            self.assertIsNotNone(grip_check_problem(cfg), autonomy_options)

    def test_autonomy3_runs_the_v3_profile(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'calib.json'
            path.write_text(json.dumps(config()))
            out = subprocess.run([sys.executable, str(ROOT / 'autonomy3.py'), '--check-config', '--config', str(path)],
                                 capture_output=True, text=True, cwd=ROOT)
            self.assertIn('Version: V3', out.stdout + out.stderr)


class FakeLink:
    def __init__(self, status):
        self.status = status

    def poll(self):
        pass

    def status_age(self):
        return 0.0 if self.status else float('inf')


class StartCheck(unittest.TestCase):
    def test_starts_only_with_a_gripper_camera(self):
        self.assertIsNone(wait_for_gripcam(FakeLink({'gripcam': 'ok'}), seconds=0.2))
        message = wait_for_gripcam(FakeLink({'gripcam': 'none'}), seconds=0.2)
        self.assertIn('gripcam=none', message)
        self.assertIn('autonomy2.py', message)
        self.assertIn('old firmware', wait_for_gripcam(FakeLink({'state': 'IDLE'}), seconds=0.2))
        self.assertIn('no firmware status', wait_for_gripcam(FakeLink(None), seconds=0.2))


class Simulator(unittest.TestCase):
    def robot(self, **params):
        return sim.SimRobot(config(), [sim.Stone(1120, 600, 2)], 1000, 600, 0, params=params)

    def test_look_reports_what_the_jaws_hold(self):
        empty = sim.SimRobot(config(), [sim.Stone(1400, 600, 2)], 1000, 600, 0)   # stone far away
        empty.command('look', 0.0, n=1)
        self.assertFalse(empty.status(0.1)['look']['done'])
        self.assertEqual(empty.status(0.5)['look'], {'n': 1, 'done': True, 'ids': [1] * 5})
        r = self.robot()
        r.command('grip', 0.5, p='close')
        for i in range(60):
            r.update(0.5 + i * 0.02)
        self.assertIsNotNone(r.held)
        r.command('look', 2.0, n=2)
        self.assertEqual(r.status(2.4)['look']['ids'], [3] * 5)

    def test_no_gripper_camera(self):
        r = self.robot(gripcam=False)
        r.command('look', 0.0, n=1)
        st = r.status(1.0)
        self.assertEqual(st['gripcam'], 'none')
        self.assertNotIn('look', st)

    def test_stone_at_the_jaw_tips_counts_as_held(self):
        r = sim.SimRobot(config(), [sim.Stone(1120 + 40, 600, 2)], 1000, 600, 0)   # 40 mm ahead
        r.command('look', 0.0, n=1)
        self.assertIsNone(r.held)
        self.assertEqual(r.status(0.5)['look']['ids'], [3] * 5)

    def test_v3_run_never_places_wrong(self):
        cfg = autonomy.prepare_config(config(), 'v3', [])
        correct = 0
        for seed in (0, 1):
            r = autonomy.run_sim(cfg, autonomy.scenario(cfg, 'scattered', seed), seconds=120,
                                 params={'grip_reach_mm': 4}, seed=seed)      # stops short: pushes
            self.assertEqual(r['wrong'], 0)
            correct += r['correct']
        self.assertGreaterEqual(correct, 6)

if __name__ == '__main__':
    unittest.main()
