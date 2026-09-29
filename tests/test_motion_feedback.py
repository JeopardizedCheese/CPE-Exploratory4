import math
from types import SimpleNamespace
import unittest

from motion_feedback import FeedbackRun, Measurement, Settings, VelocityEstimator, angle
from motion_control import clear_of_edges, ready_status


def sample(t=0, h=0, v=0, w=0, x=600, y=600):
    return Measurement(t, x, y, h, v, w)


class MotionFeedbackTests(unittest.TestCase):
    def test_small_turn_is_not_accepted_without_moving(self):
        c = FeedbackRun('turn', 1, sample(), Settings(tolerance_deg=.25))
        l, r = c.step(sample(.05))
        self.assertGreater(l, 0)
        self.assertEqual(r, -l)
        self.assertLess(l, .02)  # does not inject 71% power
        self.assertEqual(c.phase, 'running')

    def test_turn_direction_and_wraparound(self):
        for start, delta in ((179, 15), (-179, -15), (0, -15), (0, 15), (0, 180), (0, -180)):
            c = FeedbackRun('turn', delta, sample(h=start))
            l, r = c.step(sample(.05, h=start))
            self.assertEqual(math.copysign(1, l), math.copysign(1, delta))
            self.assertAlmostEqual(c.target_heading, angle(start + delta))
            self.assertEqual(r, -l)

    def test_coasts_for_predicted_arrival_and_waits_until_still(self):
        c = FeedbackRun('turn', 15, sample())
        self.assertEqual(c.step(sample(.05, h=12, w=20)), (0, 0))
        self.assertEqual(c.step(sample(.1, h=15, w=10)), (0, 0))
        self.assertNotEqual(c.phase, 'done')
        for t in range(3, 22):
            self.assertEqual(c.step(sample(t / 20, h=15)), (0, 0))
        self.assertEqual(c.phase, 'done')

    def test_lost_pose_fault_cannot_resume(self):
        c = FeedbackRun('turn', 15, sample())
        self.assertEqual(c.step(sample(.3)), (0, 0))
        self.assertEqual(c.reason, 'stale_pose')
        self.assertEqual(c.step(sample(.35)), (0, 0))

    def test_stall_fault_instead_of_claiming_success(self):
        c = FeedbackRun('turn', 15, sample())
        for k in range(1, 90):
            c.step(sample(k / 20))
        self.assertEqual(c.phase, 'aborted')
        self.assertEqual(c.reason, 'no_measured_progress_at_requested_power')

    def test_runaway_and_nan_stop(self):
        for m in (sample(.05, w=100), sample(.05, v=400), sample(.05, h=float('nan'))):
            c = FeedbackRun('turn', 15, sample())
            self.assertEqual(c.step(m), (0, 0))
            self.assertEqual(c.phase, 'aborted')

    def test_speed_saturation_and_recovery(self):
        c = FeedbackRun('speed', 100, sample(), Settings(max_duty=.3), duration=10)
        for k in range(1, 60):
            l, r = c.step(sample(k / 20, x=600 + k))
            self.assertTrue(0 <= l <= .3 and 0 <= r <= .3)
        limited_integral = c.integral
        for k in range(60, 80):
            c.step(sample(k / 20, v=180, x=600 + k))
        self.assertLessEqual(c.integral, limited_integral)
        self.assertEqual(c.last_command, (0, 0))

    def test_speed_trial_and_power_trial_end_then_settle(self):
        for mode, value in (('speed', 100), ('duty', .2)):
            c = FeedbackRun(mode, value, sample(), duration=.5)
            for k in range(1, 35):
                c.step(sample(k / 20, x=600 + min(k, 9)))
                if k >= 10:
                    self.assertEqual(c.last_command, (0, 0))
            self.assertEqual(c.phase, 'done')

    def test_invalid_settings_and_targets(self):
        for opts in ({'max_duty': float('nan')}, {'max_duty': 1.1}, {'base_duty': .6},
                     {'camera_delay_s': -1}, {'speed_ki': 0}):
            with self.assertRaises(ValueError):
                Settings(**opts)
        for mode, target in (('turn', 1), ('turn', 0), ('turn', 181), ('speed', 301),
                             ('duty', .8), ('duty', float('inf'))):
            with self.assertRaises(ValueError):
                FeedbackRun(mode, target, sample())

    def test_velocity_uses_axle_and_wraps_heading(self):
        est = VelocityEstimator(axle_offset_mm=40)
        out = None
        for k, h in enumerate((175, 178, -179, -176, -173)):
            rad = math.radians(h)
            p = SimpleNamespace(t=k * .05, heading_deg=h, x=600 + 40 * math.cos(rad),
                                y=600 + 40 * math.sin(rad))
            out = est.update(p)
        self.assertAlmostEqual(out.speed, 0, places=8)
        self.assertAlmostEqual(out.omega, 60)
        with self.assertRaises(ValueError):
            est.update(p)

    def test_old_firmware_or_wrong_session_cannot_arm(self):
        self.assertFalse(ready_status({'state': 'RUNNING'}, 'abc', .01))
        self.assertFalse(ready_status({'session': 'other', 'direct_pwm': 1}, 'abc', .01))
        self.assertFalse(ready_status({'session': 'abc', 'direct_pwm': 1}, 'abc', .7))
        self.assertTrue(ready_status({'session': 'abc', 'direct_pwm': 1}, 'abc', .1))

    def test_whole_rotating_body_must_clear_edges(self):
        cfg = {'arena': {'size_mm': [2100, 1200]}, 'robot_tag': {'axle_offset_mm': -40,
               'footprint_mm': {'front': 210, 'back': 90, 'left': 75, 'right': 75}}}
        self.assertTrue(clear_of_edges(sample(), cfg, 60))
        self.assertFalse(clear_of_edges(sample(x=220), cfg, 60))

    def test_delayed_camera_turns_in_idealized_plant(self):
        # A deliberately simple plant with no stiction. This exercises latency,
        # inertia, wraparound and both directions; it is not a hardware accuracy claim.
        for delta, gain in ((1, 80), (15, 100), (-15, 130)):
            s = Settings(tolerance_deg=.2, turn_kp=.025, timeout_s=30)
            c = FeedbackRun('turn', delta, sample(), s)
            dt, h, w = .02, 0.0, 0.0
            delay = int(s.camera_delay_s / dt)
            history = [(h, w)] * (delay + 1)
            for k in range(1, 1500):
                seen_h, seen_w = history[-delay - 1]
                l, _ = c.step(sample(k * dt, h=seen_h, w=seen_w))
                w += (l * gain - w) * min(1, dt / .08)
                h = angle(h + w * dt)
                history.append((h, w))
                if c.phase in ('done', 'aborted'):
                    break
            self.assertEqual(c.phase, 'done', (delta, gain, c.reason, c.debug))
            self.assertLessEqual(abs(angle(delta - h)), s.tolerance_deg)


if __name__ == '__main__':
    unittest.main()
