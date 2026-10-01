"""Live-tag edge recovery, bounds, mission arbitration, and send deadlines."""
import math
import time
import unittest
from dataclasses import replace
from types import SimpleNamespace

from autonomy import DriveSender, Planner
from wall_guard import DEFAULTS as WALL, WallGuard
from tests.test_autonomy import config, pose


P = WALL['wall_pulse_s']     # a drive pulse started at t=.4 by GuardTests.pulse ends at .4+P


class GuardTests(unittest.TestCase):
    def pulse(self, p, observations=(), carrying=None, cfg=None):
        guard = WallGuard(cfg or config())
        self.assertEqual(guard.step(0, replace(p, t=0)), (0, 0))
        command = guard.step(.4, replace(p, t=.4), observations, carrying)
        return guard, command

    def test_reverse_from_all_four_walls_and_outside_calibration(self):
        for x, y, heading in [(100, 600, 180), (2000, 600, 0), (1000, 100, -90),
                              (1000, 1100, 90), (-40, 600, 180), (2140, 600, 0),
                              (1000, -40, -90), (1000, 1240, 90)]:
            with self.subTest(x=x, y=y):
                guard, (l, r) = self.pulse(pose(x, y, heading))
                self.assertLess(l, 0)
                self.assertEqual(l, r)
                self.assertEqual(guard.reason, 'reverse_inward')

    def test_forward_if_already_facing_inward(self):
        guard, (l, r) = self.pulse(pose(100, 600, 0))
        self.assertGreater(l, 0)
        self.assertEqual(l, r)
        self.assertEqual(guard.reason, 'forward_inward')

    def test_diagonal_escape_from_corner(self):
        guard, (l, r) = self.pulse(pose(140, 140, -135))
        self.assertLess(l, 0)
        self.assertEqual(l, r)

    def test_tangent_heading_turns_only_with_body_clearance(self):
        # Camera margin is breached but a full rotation physically fits here.
        cfg = config()
        cfg['robot_tag']['footprint_mm'] = dict(front=135, back=90, left=75, right=75)
        guard, (l, r) = self.pulse(pose(190, 600, 90), cfg=cfg)
        self.assertEqual(guard.reason, 'turn_inward')
        self.assertLess(l*r, 0)
        guard, cmd = self.pulse(pose(60, 600, 90), cfg=cfg)
        self.assertEqual(cmd, (0, 0))
        self.assertEqual(guard.reason, 'wall_no_clear_escape')

    def test_body_not_only_tag_can_trigger_guard(self):
        cfg = config()
        cfg['robot_tag']['footprint_mm']['front'] = 300
        guard, command = self.pulse(pose(250, 600, 180), cfg=cfg)
        self.assertFalse(guard.safe_pose(pose(250, 600, 180)))
        self.assertLess(command[0], 0)

    def test_observed_stone_blocks_escape(self):
        guard, command = self.pulse(pose(100, 600, 180), [{'x': 250, 'y': 600, 'color': 2}])
        self.assertEqual(command, (0, 0))
        self.assertTrue(guard.failed)

    def test_carried_stone_is_not_pushed_into_wrong_zone(self):
        cfg = config()
        cfg['zones'] = {'4_orange': {'center_mm': [60, 600], 'radius_mm': 25}}
        guard, command = self.pulse(pose(100, 600, 180), carrying=2, cfg=cfg)
        self.assertEqual(command, (0, 0))
        self.assertTrue(guard.failed)

    def test_missing_stale_and_future_poses_cancel_pulse(self):
        for missing in (None, pose(100, 600, 180, 0), pose(100, 600, 180, 9)):
            guard, _ = self.pulse(pose(100, 600, 180))
            self.assertEqual(guard.step(.45, missing), (0, 0))
            self.assertEqual(guard.phase, 'settle')
            self.assertIsNone(guard.diagnostics()['wall_command_until'])
            self.assertEqual(guard.step(.5, pose(100, 600, 180, .5)), (0, 0))

    def test_pulse_expires_then_waits_for_new_frame(self):
        guard, _ = self.pulse(pose(100, 600, 180))
        end = .4+P
        self.assertAlmostEqual(guard.diagnostics()['wall_command_until'], end)
        self.assertEqual(guard.step(end+.01, pose(130, 600, 180, end+.01)), (0, 0))
        self.assertEqual(guard.step(end+.25, pose(130, 600, 180, end+.25)), (0, 0))
        self.assertLess(guard.step(end+.4, pose(130, 600, 180, end+.4))[0], 0)

    def test_does_not_resume_until_further_inside(self):
        guard, _ = self.pulse(pose(100, 600, 180))
        t = .4+P
        guard.step(t+.01, pose(220, 600, 180, t+.01))
        self.assertLess(guard.step(t+.4, pose(220, 600, 180, t+.4))[0], 0)
        t += .4+P
        guard.step(t+.01, pose(280, 600, 180, t+.01))
        self.assertEqual(guard.step(t+.4, pose(280, 600, 180, t+.4)), (0, 0))
        self.assertTrue(guard.completed)
        self.assertIsNone(guard.step(t+.5, pose(280, 600, 180, t+.5)))

    def test_no_progress_and_timeout_latch_stop(self):
        for finish, reason in [(3.1, 'wall_no_progress'), (12.1, 'wall_recovery_timeout')]:
            guard = WallGuard(config())
            guard.step(0, pose(100, 600, 180))
            self.assertEqual(guard.step(finish, pose(100, 600, 180, finish)), (0, 0))
            self.assertEqual(guard.reason, reason)
            self.assertEqual(guard.step(finish+.1, pose(1000, 600, 0, finish+.1)), (0, 0))

    def test_large_outward_motion_or_out_of_range_stops(self):
        guard, _ = self.pulse(pose(100, 600, 180))
        self.assertEqual(guard.step(.45, pose(50, 600, 180, .45)), (0, 0))
        self.assertEqual(guard.reason, 'wall_motion_went_outward')
        guard = WallGuard(config())
        self.assertEqual(guard.step(0, pose(-250, 600, 180)), (0, 0))
        self.assertEqual(guard.reason, 'wall_outside_recovery_range')

    def test_slow_cumulative_outward_motion_also_stops(self):
        guard, _ = self.pulse(pose(100, 600, 180))
        for i in range(1, 5):
            t = .4+i*.02
            command = guard.step(t, pose(100-i*8, 600, 180, t))
        self.assertEqual(command, (0, 0))
        self.assertEqual(guard.reason, 'wall_motion_went_outward')

    def test_forecast_stops_outward_motion_before_margin(self):
        guard = WallGuard(config())
        self.assertEqual(guard.step(0, pose(250, 600, 180), forecast=pose(150, 600, 180)), (0, 0))
        self.assertTrue(guard.active)

    def test_motion_from_before_the_stop_is_not_judged_outward(self):
        # Driving at the wall: after the stop the camera keeps showing the robot coming
        # closer (~0.2 s delay) and it coasts. That is not the recovery's doing.
        guard = WallGuard(config())
        self.assertEqual(guard.step(0, pose(250, 600, 180, 0), forecast=pose(150, 600, 180)), (0, 0))
        for i in range(1, 13):                    # 60 mm closer at ~170 mm/s, past the settle time
            t = .03*i
            self.assertEqual(guard.step(t, pose(250-5*i, 600, 180, t)), (0, 0))
        self.assertFalse(guard.failed, guard.reason)
        self.assertEqual(guard.reason, 'wall_wait_until_stopped')
        self.assertLess(guard.step(.39, pose(190, 600, 180, .39))[0], 0)   # at rest: reverse
        self.assertEqual(guard.pulses, 1)

    def test_waits_until_the_camera_shows_the_robot_stopped(self):
        guard = WallGuard(config())
        guard.step(0, pose(150, 600, 180, 0))
        for i in range(1, 21):                    # ~100 mm/s: still coasting
            t = .03*i
            self.assertEqual(guard.step(t, pose(150-3*i, 600, 180, t)), (0, 0))
        self.assertEqual((guard.reason, guard.pulses), ('wall_wait_until_stopped', 0))
        self.assertLess(guard.step(.63, pose(90, 600, 180, .63))[0], 0)

    def test_blocked_retries_after_wall_retry_s(self):
        guard, command = self.pulse(pose(100, 600, 180), [{'x': 250, 'y': 600, 'color': 2}])
        self.assertTrue(guard.failed)
        retry = .4+WALL['wall_retry_s']
        self.assertEqual(guard.step(retry-.05, pose(100, 600, 180, retry-.05)), (0, 0))
        self.assertTrue(guard.failed)
        # The stone was pushed aside: a new attempt reverses.
        self.assertEqual(guard.step(retry+.01, pose(100, 600, 180, retry+.01)), (0, 0))
        self.assertFalse(guard.failed)
        self.assertLess(guard.step(retry+.4, pose(100, 600, 180, retry+.4))[0], 0)

    def test_tangent_near_wall_turns_to_face_it_when_facing_inward_would_hit(self):
        # Field geometry: body front 135 / back 90 / sides 75, axle 40 mm ahead of the tag.
        # Parallel 160 mm from the left wall a full-circle check found no escape. Turning to
        # face inward swings the long back end into the wall; turning to face the wall
        # (then reversing out) keeps every corner >= 20 mm inside.
        cfg = config(axle=-40)
        cfg['robot_tag']['footprint_mm'] = dict(front=135, back=90, left=75, right=75)
        for heading, sign in ((90, 1), (-90, -1)):
            guard, (l, r) = self.pulse(pose(160, 600, heading), cfg=cfg)
            self.assertEqual(guard.reason, 'turn_inward')
            self.assertEqual(math.copysign(1, l), sign)     # heading moves toward 180
            self.assertAlmostEqual(guard.until, .4+WALL['wall_turn_pulse_s'])
        guard, cmd = self.pulse(pose(130, 600, 90), cfg=cfg)
        self.assertEqual((cmd, guard.reason), ((0, 0), 'wall_no_clear_escape'))

    def test_unsafe_configuration_rejected(self):
        for key, value in [('wall_pulse_s', 1), ('wall_recovery_speed', 2),
                           ('wall_pose_max_age_s', 1), ('wall_margin_mm', 600),
                           ('wall_resume_mm', float('nan')), ('wall_recovery_enabled', 'yes')]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                WallGuard(config(), {key: value})


class PlannerRecoveryTests(unittest.TestCase):
    def test_preserves_payload_and_resumes_carry_without_gripper_events(self):
        p = Planner(config())
        p.state, p.carrying, p.pick_checked = 'CARRY', 2, True
        # y=770: reversing from here does not drag the cyan stone into another zone
        for t, x in [(0, 100), (.4, 100), (.41+P, 280), (.8+P, 280)]:
            self.assertEqual(p.step(t, pose(x, 770, 180, t), [], [])[2], [])
        self.assertEqual(p.state, 'CARRY')
        self.assertEqual(p.carrying, 2)

    def test_backoff_is_interrupted_and_old_target_skipped_after_recovery(self):
        p = Planner(config())
        p.state = 'BACKOFF'
        p.lock.target = {'x': 300, 'y': 600, 'color': 2, 'confidence': .9}
        p._pulse = {'end': 9, 'settle': 10}
        for t, x in [(0, 100), (.4, 100), (.41+P, 280), (.8+P, 280)]:
            p.step(t, pose(x, 600, 180, t), [], [])
        self.assertEqual(p.state, 'SEARCH')
        self.assertIsNone(p._pulse)
        self.assertIsNone(p.lock.target)
        self.assertTrue(p.skip)

    def test_firmware_and_vision_gates_cancel_recovery_pulses(self):
        for fields in ({'status': {'state': 'IDLE'}}, {'require_status': True},
                       {'perception_status': 'lighting_change'}):
            p = Planner(config())
            p.step(0, pose(100, 600, 180), [], [])
            self.assertLess(p.step(.4, pose(100, 600, 180, .4), [], [])[0], 0)
            self.assertEqual(p.step(.45, pose(100, 600, 180, .45), [], [], **fields), (0, 0, []))
            self.assertEqual(p.wall.phase, 'settle')
            self.assertIsNone(p.debug['wall_command_until'])

    def test_grip_and_release_finish_while_stationary_before_recovery(self):
        for state, target, next_state in [('GRIP', 70, 'CARRY'), ('RELEASE', 0, 'BACKOFF')]:
            p = Planner(config())
            p.state, p.carrying = state, 2
            result = p.step(.1, pose(100, 600, 180, .1), [], [], {'state': 'RUNNING', 'servo': [target]})
            self.assertEqual(result, (0, 0, []))
            self.assertEqual(p.state, next_state)
            p.step(.2, pose(100, 600, 180, .2), [], [])
            self.assertEqual(p.state, 'WALL_RECOVERY')

    def test_retry_that_finds_the_robot_safe_resumes_the_mission(self):
        p = Planner(config())
        p.state, p.carrying, p.pick_checked = 'CARRY', 2, True
        p.step(0, pose(100, 600, 180, 0), [], [{'x': 250, 'y': 600, 'color': 0}])
        p.step(.4, pose(100, 600, 180, .4), [], [{'x': 250, 'y': 600, 'color': 0}])
        self.assertEqual(p.state, 'WALL_BLOCKED')
        t = .4+WALL['wall_retry_s']+.1
        p.step(t, pose(600, 770, 180, t), [], [])            # moved back inside by hand
        self.assertEqual(p.state, 'CARRY')
        self.assertEqual(p.carrying, 2)

    def test_staging_and_park_stay_inside(self):
        cfg = config()
        cfg['autonomy'] = {'park_mm': [-100, 1400]}
        p = Planner(cfg)
        self.assertGreaterEqual(p.park[0], 200)
        self.assertLessEqual(p.park[1], 1000)
        # A pickable stone can still have an impossible staging point.
        target = {'x': 350, 'y': 600, 'color': 2, 'confidence': .9, 'approach_deg': 0}
        self.assertIsNone(p._choose(pose(600, 600, 0))([target]))
        target['approach_deg'] = 180
        self.assertIsNotNone(p._choose(pose(600, 600, 0))([target]))

    def test_closed_loop_outside_field_with_live_camera_poses(self):
        # Ideal measured kinematics, not a claim of calibrated motor speed.
        for start in [(-40, 600, 180), (2140, 600, 0), (1000, -40, -90), (1000, 1240, 90)]:
            with self.subTest(start=start):
                p = Planner(config())
                x, y, heading = start
                seen = False
                for i in range(601):
                    t = i*.02
                    l, r, events = p.step(t, pose(x, y, heading, t), [], [])
                    self.assertEqual(events, [])
                    seen |= p.state == 'WALL_RECOVERY'
                    if seen and p.state == 'SEARCH':
                        break
                    speed = (l+r)/(.18*2)*400  # 400 mm/s while wheels pulse
                    x += speed*math.cos(math.radians(heading))*.02
                    y += speed*math.sin(math.radians(heading))*.02
                self.assertEqual(p.state, 'SEARCH', p.debug)
                self.assertGreaterEqual(min(p.wall.clearances(pose(x, y, heading), 60)), 0)

    def test_charged_physics_returns_from_wall_with_camera_latency(self):
        import sim
        cfg = config()
        # User's tag remains visible in the raw camera, unlike the old field preset.
        robot = sim.SimRobot(cfg, [], 100, 600, 180, params=dict(sim.FIELD_PARAMS, tag_edge_mm=0))
        robot.command('start', 0)
        p = Planner(cfg)
        seen = False
        for i in range(601):
            t = i*.02
            robot.update(t)
            l, r, events = p.step(t, robot.perceive(t)[0], [], [], robot.status(t))
            self.assertEqual(events, [])
            robot.command('drive', t, l=l, r=r)
            seen |= p.state == 'WALL_RECOVERY'
            if seen and p.state == 'SEARCH':
                break
        self.assertEqual(p.state, 'SEARCH', p.debug)
        self.assertGreaterEqual(robot.x, 260)


class SenderDeadlineTests(unittest.TestCase):
    def test_pulse_deadline_stops_sends_even_if_camera_processing_stalls(self):
        sent = []
        sender = DriveSender(SimpleNamespace(send=lambda cmd, **f: sent.append((time.monotonic(), f))),
                             rate=200, stale_s=.5)
        try:
            deadline = time.monotonic()+.08
            sender.set(-.18, -.18, valid_until=deadline)
            time.sleep(.16)
        finally:
            sender.stop()
        self.assertTrue(any(f['l'] != 0 for _, f in sent))
        self.assertTrue(any(t >= deadline for t, _ in sent))
        self.assertTrue(all(f['l'] == f['r'] == 0 for t, f in sent if t >= deadline))


if __name__ == '__main__':
    unittest.main()
