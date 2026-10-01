import math
import unittest

from gesture_logic import Hand
from lever_logic import LeverControl
from lever_link import LeverSession


class LeverTests(unittest.TestCase):
    def setUp(self):
        self.control = LeverControl(.2)
        self.control.start()
        self.t = 10.

    def frame(self, pose='FIST', x=.5, y=.5, dt=.05):
        self.t += dt
        hands = [] if pose is None else [Hand(self.t, pose, x, y)]
        return self.control.update(hands, self.t)

    def hold(self, pose='FIST', x=.5, y=.5, n=22):
        return [e for _ in range(n) for e in self.frame(pose, x, y)[2]]

    def drive(self):
        self.hold()
        self.assertEqual(self.control.mode, 'DRIVE')

    def servo(self):
        self.drive()
        self.assertEqual(self.hold('THUMB_UP'), [])
        self.assertEqual(self.control.mode, 'SERVO')
        self.assertTrue(self.control.servo_ready)

    def test_paused_cannot_arm_drive_or_grip(self):
        self.control.pause()
        self.hold()
        self.assertEqual(self.hold('THUMB_UP', y=.2), [])
        self.assertEqual(self.frame(y=.2), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')

    def test_requires_center_and_stable_fist_before_motion(self):
        self.hold(x=.8)
        self.assertEqual(self.control.mode, 'STOP')
        for i in range(40):
            self.frame(x=.45 if i % 2 else .55)
        self.assertEqual(self.control.mode, 'STOP')
        self.drive()
        self.assertEqual(self.frame(y=.25), (.2, .2, []))

    def test_reference_is_calibrated_to_held_fist(self):
        self.hold(x=.56, y=.54)
        self.assertEqual(self.control.center, (.56, .54))
        self.assertEqual(self.frame(x=.56, y=.54), (0., 0., []))
        self.assertEqual(self.frame(x=.56, y=.3)[:2], (.2, .2))

    def test_four_directions_and_neutral(self):
        self.drive()
        self.assertEqual(self.frame(y=.25)[:2], (.2, .2))
        self.assertEqual(self.frame(y=.75)[:2], (-.2, -.2))
        self.assertEqual(self.frame(x=.75)[:2], (.12, -.12))
        self.assertEqual(self.frame(x=.25)[:2], (-.12, .12))
        self.assertEqual(self.frame(x=.55, y=.55), (0., 0., []))

    def test_stop_poses_and_loss_cancel_drive_and_require_center(self):
        for pose in ('OPEN', 'UNKNOWN', 'V', 'ONE', 'THUMB_DOWN', None):
            with self.subTest(pose=pose):
                self.drive()
                self.frame(y=.25)
                self.assertEqual(self.frame(pose), (0., 0., []))
                self.assertEqual(self.control.mode, 'STOP')
                self.assertEqual(self.frame(y=.25), (0., 0., []))

    def test_multiple_hands_stop_instead_of_selecting_one(self):
        self.drive()
        self.t += .05
        hands = [Hand(self.t, 'FIST', .5, .25), Hand(self.t, 'THUMB_UP', .2, .5)]
        self.assertEqual(self.control.update(hands, self.t), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')

    def test_frozen_frame_cannot_finish_arming(self):
        hand = Hand(self.t, 'FIST')
        for elapsed in (.01, .05, .1, .15, .19, .21, .8):
            self.assertEqual(self.control.update([hand], self.t+elapsed), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')

    def test_stale_gap_and_reordered_frames_cancel(self):
        self.drive()
        self.frame(y=.25)
        hand = Hand(self.t, 'FIST', .5, .25)
        self.assertEqual(self.control.update([hand], self.t+.21), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')
        self.t += .3
        self.drive()
        self.assertEqual(self.frame(y=.25, dt=.5), (0., 0., []))
        self.drive()
        self.assertEqual(self.control.update([Hand(self.t-.01, 'FIST')], self.t), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')

    def test_bad_coordinates_and_timestamps_cannot_arm(self):
        for hand in (Hand(self.t, 'FIST', math.nan), Hand(self.t, 'FIST', y=2),
                     Hand(self.t+1, 'FIST'), Hand(math.inf, 'FIST')):
            self.assertEqual(self.control.update([hand], self.t), (0., 0., []))
            self.assertEqual(self.control.mode, 'STOP')

    def test_thumb_immediately_stops_drive_before_entry_dwell(self):
        self.drive()
        self.frame(y=.25)
        self.assertEqual(self.frame('THUMB_UP', y=.25), (0., 0., []))
        self.assertEqual(self.control.mode, 'SERVO_ENTRY')
        self.assertEqual(self.frame('FIST', y=.25), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')

    def test_servo_entry_references_new_position_without_gripping(self):
        self.drive()
        self.assertEqual(self.hold('THUMB_UP', x=.55, y=.4), [])
        self.assertEqual(self.control.center, (.55, .4))
        self.assertEqual(self.control.mode, 'SERVO')
        self.assertEqual(self.frame('THUMB_UP', x=.55, y=.4), (0., 0., []))

    def test_servo_center_cannot_be_calibrated_at_image_edge(self):
        self.drive()
        self.assertEqual(self.hold('THUMB_UP', y=.05), [])
        self.assertEqual(self.control.mode, 'SERVO_ENTRY')
        self.assertEqual(self.hold('THUMB_UP'), [])
        self.assertEqual(self.control.mode, 'SERVO')

    def test_servo_must_settle_after_entry_before_accepting_action(self):
        self.drive()
        for _ in range(20):
            self.frame('THUMB_UP')
            if self.control.mode == 'SERVO_CENTER':
                break
        self.assertEqual(self.control.mode, 'SERVO_CENTER')
        self.assertEqual(self.hold('THUMB_UP', y=.25), [])
        self.assertEqual(self.control.mode, 'SERVO_CENTER')
        self.hold('THUMB_UP')
        self.assertEqual(self.hold('THUMB_UP', y=.25), [('grip', 'close')])

    def test_close_once_and_require_neutral_before_open(self):
        self.servo()
        self.assertEqual(self.hold('THUMB_UP', y=.25), [('grip', 'close')])
        self.assertEqual(self.hold('THUMB_UP', y=.25, n=100), [])
        self.assertEqual(self.hold('THUMB_UP', y=.75), [])
        self.hold('THUMB_UP')
        self.assertEqual(self.hold('THUMB_UP', y=.75), [('grip', 'open')])

    def test_short_jitter_and_sideways_servo_motion_do_nothing(self):
        self.servo()
        self.assertEqual(self.hold('THUMB_UP', x=.8), [])
        for i in range(30):
            self.assertEqual(self.frame('THUMB_UP', y=.25 if i % 2 else .5), (0., 0., []))

    def test_all_servo_states_lock_wheels_and_exit_keeps_grip(self):
        self.drive()
        for y in [.5]*22 + [.25]*10 + [.5]*10 + [.75]*10:
            self.assertEqual(self.frame('THUMB_UP', y=y)[:2], (0., 0.))
        self.assertEqual(self.frame('OPEN'), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')
        self.assertEqual(self.frame('FIST', y=.25), (0., 0., []))

    def test_fold_thumb_does_not_resume_displaced_drive(self):
        self.servo()
        self.assertEqual(self.frame('FIST', y=.25), (0., 0., []))
        self.assertEqual(self.control.mode, 'STOP')
        self.assertEqual(self.hold('FIST', y=.25), [])
        self.assertEqual(self.control.mode, 'STOP')

    def test_invalid_speed_rejected(self):
        for speed in (0, .01, -.1, 1.1, math.nan, math.inf):
            with self.assertRaises(ValueError):
                LeverControl(speed)

    def test_speed_keys_step_clamp_and_apply_at_once(self):
        self.drive()
        self.assertEqual(self.control.adjust_speed(+1), .25)
        self.assertEqual(self.frame(y=.25)[:2], (.25, .25))
        self.assertEqual(self.control.adjust_speed(-1), .2)
        for _ in range(30):
            self.control.adjust_speed(+1)
        self.assertEqual(self.control.speed, 1.)
        for _ in range(30):
            self.control.adjust_speed(-1)
        self.assertEqual(self.control.speed, .02)        # lowest valid speed, as --speed
        self.assertEqual(self.control.mode, 'DRIVE')     # changing speed never disarms


class FakeLink:
    def __init__(self):
        self.session = '0123456789ab'
        self.status = None
        self.status_at = 0.
        self.packets = []
        self.closed = False

    def send(self, command, **fields):
        self.packets.append((command, fields))

    def poll(self):
        pass

    def running(self, now):
        return bool(self.status and self.status['state'] == 'RUNNING' and 0 <= now-self.status_at < .6)

    def close(self):
        self.closed = True


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.control = LeverControl(.2)
        self.link = FakeLink()
        self.session = LeverSession(self.control, self.link)
        self.t = 10.

    def status(self, session=None, state='RUNNING'):
        self.link.status = {'state': state, 'session': session or self.link.session,
                            **getattr(self.link, 'status_extra', {})}
        self.link.status_at = self.t

    def frame(self, pose='FIST', y=.5, dt=.05, status=True):
        self.t += dt
        if status:
            self.status()
        hands = [] if pose is None else [Hand(self.t, pose, .5, y)]
        return self.session.tick(hands, self.t)

    def hold(self, pose='FIST', y=.5, n=22):
        return [self.frame(pose, y) for _ in range(n)]

    def drive(self):
        self.session.start(self.t)
        self.hold()
        self.assertEqual(self.control.mode, 'DRIVE')

    def test_preview_needs_no_link_and_starts_locally(self):
        s = LeverSession(self.control)
        s.start(self.t)
        self.assertTrue(self.control.enabled)
        self.assertEqual(s.tick([], self.t), (0., 0., []))
        s.close()
        self.assertFalse(self.control.enabled)

    def test_cached_or_other_session_running_cannot_start(self):
        self.status()
        self.session.start(self.t)
        self.assertIsNone(self.link.status)
        self.t += .1
        self.status(session='wrong_session')
        self.session.tick([Hand(self.t, 'FIST')], self.t)
        self.assertFalse(self.control.enabled)
        self.t += .1
        self.status()
        self.session.tick([Hand(self.t, 'FIST')], self.t)
        self.assertTrue(self.control.enabled)
        self.assertEqual(self.control.mode, 'STOP')

    def test_start_retried_then_times_out_stopped(self):
        self.session.start(self.t)
        self.session.tick([], self.t+.3)
        self.assertEqual(sum(c == 'start' for c, _ in self.link.packets), 2)
        self.session.tick([], self.t+1.6)
        self.assertFalse(self.control.enabled)
        self.assertIsNone(self.session.pending_start)
        self.assertEqual(self.link.packets[-1][0], 'stop')

    def test_status_loss_stops_and_cannot_auto_resume(self):
        self.drive()
        self.frame(y=.25)
        self.frame(y=.25, dt=.7, status=False)
        self.assertFalse(self.control.enabled)
        self.assertEqual(self.link.packets[-1][0], 'stop')
        self.hold()
        self.assertFalse(self.control.enabled)

    def test_session_change_stops_control(self):
        self.drive()
        self.t += .05
        self.status(session='other')
        self.session.tick([Hand(self.t, 'FIST', y=.25)], self.t)
        self.assertFalse(self.control.enabled)

    def test_stop_packet_bypasses_20_hz_interval(self):
        self.drive()
        self.frame(y=.25)
        self.link.packets.clear()
        self.frame('THUMB_UP', y=.25, dt=.001)
        self.assertIn(('drive', {'l': 0., 'r': 0.}), self.link.packets)

    def test_grip_retried_three_times_with_zero_wheels_first(self):
        self.drive()
        self.hold('THUMB_UP')
        self.link.packets.clear()
        self.hold('THUMB_UP', y=.25)
        grips = [(i, p) for i, p in enumerate(self.link.packets) if p[0] == 'grip']
        self.assertEqual(len(grips), 3)
        for i, packet in grips:
            self.assertEqual(packet, ('grip', {'p': 'close'}))
            self.assertEqual(self.link.packets[i-1], ('drive', {'l': 0., 'r': 0.}))
        self.assertFalse(any(c == 'drive' and (f['l'] or f['r']) for c, f in self.link.packets))

    def test_hand_loss_cancels_queued_grip_retries(self):
        self.drive()
        self.hold('THUMB_UP')
        for _ in range(10):
            _, _, events = self.frame('THUMB_UP', y=.25)
            if events:
                break
        self.assertIsNotNone(self.session.grip_retry)
        self.link.packets.clear()
        self.frame(None)
        self.assertIsNone(self.session.grip_retry)
        self.assertFalse(any(c == 'grip' for c, _ in self.link.packets))

    def test_close_stops_and_closes_socket(self):
        self.drive()
        self.session.close()
        self.assertTrue(self.link.closed)
        self.assertFalse(self.control.enabled)
        self.assertEqual(self.link.packets[-1], ('stop', {}))


    def floor_session(self, report=True):
        self.session = LeverSession(self.control, self.link, min_duty=.65)
        self.report = report

    def test_min_duty_sent_with_every_drive_packet(self):
        self.floor_session()
        self.link.status_extra = {'min_duty': .65}
        self.drive()
        self.frame(y=.25)
        self.frame(y=.5)
        drives = [f for c, f in self.link.packets if c == 'drive']
        self.assertTrue(drives)
        self.assertTrue(all(f.get('m') == .65 for f in drives))
        self.assertIn(('drive', {'l': .2, 'r': .2, 'm': .65}), self.link.packets)

    def test_default_sends_no_floor(self):
        self.drive()
        self.frame(y=.25)
        self.assertFalse(any('m' in f for c, f in self.link.packets if c == 'drive'))

    def test_min_duty_with_firmware_that_does_not_report_it_stops(self):
        self.floor_session()
        self.session.start(self.t)
        self.hold()
        self.assertFalse(self.control.enabled)
        self.assertIn('min_duty', self.session.message)
        self.assertFalse(any(c == 'drive' and (f['l'] or f['r']) for c, f in self.link.packets))

    def test_invalid_min_duty_rejected(self):
        for floor in (-.1, 1.1, math.nan):
            with self.assertRaises(ValueError):
                LeverSession(self.control, self.link, min_duty=floor)


if __name__ == '__main__':
    unittest.main()
