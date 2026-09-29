"""Progress past turning, retention of one goal, and observable stop conditions."""
from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

import autonomy
from autonomy import Planner
import sim
from tests.test_autonomy import config, pose, NOISE
from tests.test_vision import configuration


def target(x, y, color=2):
    return {'color': color, 'x': x, 'y': y, 'confidence': .9}


class ProgressTests(unittest.TestCase):
    def test_near_stage_forward_corrections_never_reverse_a_wheel(self):
        p = Planner(config())
        for distance in (40, 80, 160, 500):
            for degrees in (-34, -20, -10, 0, 10, 20, 34):
                with self.subTest(distance=distance, degrees=degrees):
                    angle = math.radians(degrees)
                    l, r = p._drive_to(pose(0, 0, 0), 0, 0,
                                       distance*math.cos(angle), distance*math.sin(angle), p.o['cruise'])
                    self.assertGreater(l, .01)  # firmware zero-command cutoff
                    self.assertGreater(r, .01)
                    self.assertEqual(p.debug['reason'], 'drive_forward')

    def test_large_error_still_turns_in_place(self):
        p = Planner(config())
        l, r = p._drive_to(pose(0, 0, 0), 0, 0, 0, 200, p.o['cruise'])
        self.assertGreater(l, 0)
        self.assertLess(r, 0)
        self.assertEqual(p.debug['reason'], 'turn_to_goal')

    def test_close_stone_hidden_by_long_gripper_advances_instead_of_turning_back(self):
        cfg = config()
        cfg['robot_tag'].update(grip_offset_mm=[270, 0],
                                footprint_mm={'front': 300, 'back': 130, 'left': 120, 'right': 120})
        p = Planner(cfg)
        stone = target(890, 600)  # only 20 mm in front of the jaws, inside the body mask
        p.lock.update([stone], 0)
        p.state, p.heading = 'GOTO_STAGE', 0.0
        self.assertEqual(p.step(2, pose(600, 600, 0, 2, grip=270), [], [])[:2], (0, 0))
        self.assertEqual(p.state, 'ALIGN')
        self.assertEqual(p.lock.reason, 'occluded')
        p.step(2.1, pose(600, 600, 0, 2.1, grip=270), [], [], {'state': 'RUNNING', 'servo': [0]})
        self.assertEqual(p.state, 'APPROACH')
        l, r, _ = p.step(2.2, pose(600, 600, 0, 2.2, grip=270), [], [])
        self.assertGreater(l, 0)
        self.assertGreater(r, 0)

    def test_lost_target_returns_to_search_before_acquiring_new_heading(self):
        p = Planner(config())
        p.lock.update([target(1200, 600)], 0)
        p.state, p.heading = 'GOTO_STAGE', 0.0
        other = target(600, 1000, 3)
        l, r, _ = p.step(2, pose(400, 600, 0, 2), [other], [other])
        self.assertEqual((l, r), (0, 0))
        self.assertEqual(p.state, 'SEARCH')
        self.assertIsNone(p.lock.target)
        p.step(2.1, pose(400, 600, 0, 2.1), [other], [other])
        self.assertEqual(p.lock.target['color'], 3)
        self.assertAlmostEqual(p.heading, math.atan2(400, 200))

    def test_creep_tuning_does_not_reverse_a_wheel(self):
        cfg = config()
        cfg['autonomy'] = {'creep': .05}
        p = Planner(cfg)
        p.lock.update([target(800, 620)], 0)
        p.state, p.heading = 'APPROACH', 0.0
        l, r, _ = p.step(.1, pose(600, 600, 0, .1), [], [])
        self.assertGreater(l, 0)
        self.assertGreater(r, 0)

    def test_invalid_vision_revokes_commands_even_with_locked_goal(self):
        p = Planner(config())
        stone = target(1200, 600)
        p.lock.update([stone], 0)
        p.state = 'GOTO_STAGE'
        for status in ('setup_required', 'reference_moved', 'reference_unverified', 'lighting_or_camera_change'):
            self.assertEqual(p.step(.1, pose(400, 600, 0, .1), [stone], [stone],
                                    perception_status=status), (0, 0, []))
            self.assertEqual(p.debug['reason'], 'vision_' + status)

    def test_missing_pose_and_real_status_have_explicit_stop_reasons(self):
        p = Planner(config())
        self.assertEqual(p.step(1, None, [], []), (0, 0, []))
        self.assertEqual(p.debug['reason'], 'tag_missing')
        self.assertEqual(p.step(1, pose(0, 0, 0, 0), [], []), (0, 0, []))
        self.assertEqual(p.debug['reason'], 'pose_stale')
        self.assertEqual(p.step(1, pose(0, 0, 0, 1), [], [], require_status=True), (0, 0, []))
        self.assertEqual(p.debug['reason'], 'no_fresh_firmware_status')

    def test_align_waiting_for_open_gripper_is_reported(self):
        p = Planner(config())
        p.lock.update([target(1000, 600)], 0)
        p.state, p.heading = 'ALIGN', 0.0
        self.assertEqual(p.step(.1, pose(600, 600, 0, .1), [], [],
                                {'state': 'RUNNING', 'servo': [40]}), (0, 0, []))
        self.assertEqual(p.debug['reason'], 'waiting_gripper_open')

    def test_single_stone_with_270mm_gripper_completes_in_clean_and_noisy_simulation(self):
        cfg = config()
        cfg['robot_tag'].update(grip_offset_mm=[270, 0],
                                footprint_mm={'front': 300, 'back': 130, 'left': 120, 'right': 120})
        for params in (None, dict(NOISE, grip_success=1.0)):
            result = autonomy.run_sim(cfg, [sim.Stone(1100, 600, 2)], seconds=25,
                                      start=(600, 600, 90), params=params)
            self.assertEqual(result['correct'], 1)
            self.assertEqual(result['wrong'], 0)
            self.assertIn('APPROACH', [row[2] for row in result['log']])


class RealRunnerTests(unittest.TestCase):
    def test_uncalibrated_profile_reports_missing_runtime_calibration(self):
        cfg = json.loads((Path(__file__).resolve().parents[1] / 'calib.json').read_text())
        cfg['hsv'], cfg['zones'] = {}, {}
        with tempfile.TemporaryDirectory() as d:                   # no background.png next to it
            problems = autonomy.setup_problems(cfg, Path(d) / 'calib.json')
        self.assertTrue(any('HSV' in p for p in problems))
        self.assertTrue(any('zones' in p for p in problems))
        self.assertTrue(any('reference' in p for p in problems))

    def test_dry_run_logs_decisions_without_constructing_a_robot_link(self):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            cfg = config()
            cfg['hsv'] = configuration()['hsv']
            cfg['arena']['corners_px'] = [[0, 0], [23, 0], [23, 23], [0, 23]]
            config_path = folder / 'calib.json'
            config_path.write_text(json.dumps(cfg))
            raw = np.full((24, 24, 3), 180, np.uint8)
            cv2.imwrite(str(folder / 'background.png'), raw)
            args = SimpleNamespace(config=config_path, camera=1, esp_ip=None, port=4211,
                                   headless=True, dry_run=True, log_dir=folder / 'logs')
            snap = SimpleNamespace(frame=raw, pose=pose(600, 600, 0, time.monotonic()),
                                   status='ok', targets=[target(1100, 600)],
                                   observations=[{'color': 2, 'x': 1100, 'y': 600}, {'color': 0, 'x': 900, 'y': 300}])
            with patch('cv2.VideoCapture'), patch('cv2.destroyAllWindows'), \
                    patch('detect_live.LatestFrame') as reader, patch('perception.Perception') as perception, \
                    patch('teleop.Link') as link, patch('autonomy.DriveSender') as sender, redirect_stdout(io.StringIO()):
                reader.return_value.read.side_effect = [(True, raw), (False, None)]
                perception.return_value.step.return_value = snap
                perception.return_value.pose_est.last_reason = 'ok'
                autonomy.run_real(args, cfg)
                link.assert_not_called()
                sender.assert_not_called()
            log = next((folder / 'logs').glob('*/trace.jsonl'))
            row = json.loads(log.read_text().strip())
            self.assertTrue(row['dry_run'])
            self.assertEqual(row['state'], 'GOTO_STAGE')
            self.assertEqual(row['events'], [['grip', {'p': 'open'}]])
            self.assertEqual(row['observation_list'], snap.observations)
            self.assertEqual(row['target_list'], snap.targets)


class DriveFloorTests(unittest.TestCase):
    """autonomy min_duty: the floor duty sent with every drive packet."""

    def run_real_with_status(self, firmware_status, min_duty=.5):
        with tempfile.TemporaryDirectory() as d:
            folder = Path(d)
            cfg = config()
            cfg['hsv'] = configuration()['hsv']
            cfg['arena']['corners_px'] = [[0, 0], [23, 0], [23, 23], [0, 23]]
            cfg['autonomy'] = {'min_duty': min_duty}
            config_path = folder / 'calib.json'
            config_path.write_text(json.dumps(cfg))
            raw = np.full((24, 24, 3), 180, np.uint8)
            cv2.imwrite(str(folder / 'background.png'), raw)
            args = SimpleNamespace(config=config_path, camera=1, esp_ip='127.0.0.1', port=4211,
                                   headless=True, dry_run=False, log_dir=folder / 'logs')
            snap = SimpleNamespace(frame=raw, pose=pose(600, 600, 0, time.monotonic()), status='ok',
                                   targets=[], observations=[])
            out = io.StringIO()
            with patch('cv2.VideoCapture'), patch('cv2.destroyAllWindows'), \
                    patch('detect_live.LatestFrame') as reader, patch('perception.Perception') as perception, \
                    patch('teleop.Link') as link, patch('autonomy.DriveSender') as sender, redirect_stdout(out):
                reader.return_value.read.side_effect = [(True, raw), (False, None)]
                perception.return_value.step.return_value = snap
                perception.return_value.pose_est.last_reason = 'ok'
                link.return_value.status = firmware_status
                link.return_value.status_age.return_value = 0.0
                autonomy.run_real(args, cfg)
            rows = (next((folder / 'logs').glob('*/trace.jsonl'))).read_text().splitlines()
            return rows, sender.call_args, out.getvalue()

    def test_floor_is_sent_and_logged_with_supporting_firmware(self):
        rows, sender_call, printed = self.run_real_with_status({'state': 'RUNNING', 'servo': [0], 'min_duty': .5})
        self.assertEqual(sender_call.kwargs['floor'], {'m': .5})
        self.assertEqual(len(rows), 1)
        self.assertEqual(json.loads(rows[0])['firmware']['min_duty'], .5)
        self.assertIn('min_duty 0.5', printed)

    def test_old_firmware_without_min_duty_stops_before_planning(self):
        rows, _, printed = self.run_real_with_status({'state': 'RUNNING', 'servo': [0]})
        self.assertEqual(rows, [])
        self.assertIn('does not report min_duty', printed)

    def test_no_floor_keeps_old_packets_and_old_firmware(self):
        rows, sender_call, _ = self.run_real_with_status({'state': 'RUNNING', 'servo': [0]}, min_duty=None)
        self.assertEqual(sender_call.kwargs['floor'], {})
        self.assertEqual(len(rows), 1)

    def test_drive_sender_adds_floor_to_every_drive_packet(self):
        for floor in ({'m': .45}, {}):
            sent = []
            link = SimpleNamespace(send=lambda cmd, **f: sent.append((cmd, f)))
            sender = autonomy.DriveSender(link, rate=200, floor=floor)
            sender.set(.2, -.2)
            time.sleep(.05)
            sender.stop()
            drives = [f for cmd, f in sent if cmd == 'drive']
            self.assertTrue(drives)
            self.assertTrue(all({k: f[k] for k in f if k == 'm'} == floor for f in drives))

    def test_simulated_run_uses_the_floor(self):
        cfg = config()
        cfg['autonomy'] = {'min_duty': .6}
        result = autonomy.run_sim(cfg, [sim.Stone(1100, 600, 2)], seconds=2, start=(600, 600, 0))
        self.assertEqual(result['robot'].floor, .6)

    def test_set_overrides_and_min_duty_check(self):
        cfg = {'autonomy': {'cruise': .3}}
        autonomy.apply_overrides(cfg, ['min_duty=0.5', 'turn_mode=fixed', 'timeouts_s={"ALIGN": 8}',
                                       'park_mm=null'])
        self.assertEqual(cfg['autonomy'], {'cruise': .3, 'min_duty': .5, 'turn_mode': 'fixed',
                                           'timeouts_s': {'ALIGN': 8}, 'park_mm': None})
        self.assertEqual(Planner(dict(config(), autonomy=cfg['autonomy'])).o['timeouts_s']['ALIGN'], 8)
        with self.assertRaises(ValueError):
            autonomy.apply_overrides(cfg, ['min_dutty=0.5'])          # typo: never silently ignored
        with self.assertRaises(ValueError):
            autonomy.apply_overrides(cfg, ['min_duty'])
        self.assertIsNone(autonomy.min_duty_problem(cfg))
        for bad in (1.5, -.1, 'high', True):
            self.assertIsNotNone(autonomy.min_duty_problem({'autonomy': {'min_duty': bad}}))
        self.assertIsNone(autonomy.min_duty_problem({'autonomy': {'min_duty': None}}))


if __name__ == '__main__':
    unittest.main()
