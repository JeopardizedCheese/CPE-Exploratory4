import importlib.util
import math
import unittest
from unittest.mock import patch

from gesture_logic import (GRIP_HOLD_S, MOVE_HOLD_S, TURN_RATIO, GestureDrive, Hand, combine, duty)
from gesture_link import GestureSession


class CombineTests(unittest.TestCase):
    def setUp(self):
        self.t = 10.

    def hands(self, *poses, age=0.):
        return [Hand(self.t - age, pose, .3 + .4*i, .5) for i, pose in enumerate(poses)]

    def test_either_hand_alone_gives_the_command(self):
        self.assertEqual(combine(self.hands('FORWARD'), self.t, .2), ('FORWARD', (0,)))
        self.assertEqual(combine(self.hands('NONE', 'LEFT'), self.t, .2), ('LEFT', (1,)))
        self.assertEqual(combine(self.hands('LEFT', 'NONE'), self.t, .2), ('LEFT', (0,)))

    def test_same_command_on_both_hands_is_fine(self):
        self.assertEqual(combine(self.hands('BACK', 'BACK'), self.t, .2), ('BACK', (0, 1)))

    def test_stop_wins_and_disagreement_stops(self):
        self.assertEqual(combine(self.hands('FORWARD', 'STOP'), self.t, .2)[0], 'STOP')
        self.assertEqual(combine(self.hands('FORWARD', 'RIGHT'), self.t, .2)[0], 'CONFLICT')
        self.assertEqual(combine(self.hands('GRIP_OPEN', 'FORWARD'), self.t, .2)[0], 'CONFLICT')

    def test_no_hand_none_stale_and_bad_values(self):
        self.assertEqual(combine([], self.t, .2)[0], 'NO_HAND')
        self.assertEqual(combine(self.hands('NONE', 'NONE'), self.t, .2)[0], 'NONE')
        self.assertEqual(combine(self.hands('FORWARD', age=.25), self.t, .2)[0], 'NO_HAND')
        self.assertEqual(combine([Hand(self.t, 'FORWARD', math.nan, .5)], self.t, .2)[0], 'NO_HAND')
        self.assertEqual(combine([Hand(self.t + 1, 'FORWARD')], self.t, .2)[0], 'NO_HAND')
        self.assertEqual(combine(self.hands('UNKNOWN_LABEL'), self.t, .2)[0], 'NONE')


class DriveTests(unittest.TestCase):
    def setUp(self):
        self.control = GestureDrive(.5)
        self.control.start()
        self.t = 10.

    def frame(self, *poses, dt=.05):
        self.t += dt
        return self.control.update([Hand(self.t, p, .3 + .4*i, .5) for i, p in enumerate(poses)], self.t)

    def hold(self, *poses, seconds=.5):
        out = []
        for _ in range(int(round(seconds / .05))):
            out.append(self.frame(*poses))
        return out

    def test_paused_never_moves_or_grips(self):
        self.control.pause()
        self.assertTrue(all(o == (0., 0., []) for o in self.hold('FORWARD') + self.hold('GRIP_CLOSE')))
        self.assertEqual(self.control.command, 'GRIP_CLOSE')   # still shown on screen

    def test_motion_needs_a_steady_hold_then_drives(self):
        first = self.frame('FORWARD')
        self.assertEqual(first, (0., 0., []))
        out = self.hold('FORWARD', seconds=MOVE_HOLD_S + .05)
        self.assertEqual(out[-1], (.5, .5, []))

    def test_four_directions(self):
        for pose, wheels in (('FORWARD', (.5, .5)), ('BACK', (-.5, -.5)),
                             ('LEFT', (-.5*TURN_RATIO, .5*TURN_RATIO)), ('RIGHT', (.5*TURN_RATIO, -.5*TURN_RATIO))):
            self.hold('NONE', seconds=.1)
            self.assertEqual(self.hold(pose)[-1][:2], wheels, pose)

    def test_anything_else_stops_at_once(self):
        for interrupt in (('NONE',), ('STOP',), (), ('FORWARD', 'LEFT'), ('FORWARD', 'STOP'), ('GRIP_OPEN',)):
            self.hold('FORWARD')
            self.assertNotEqual(self.frame('FORWARD')[:2], (0., 0.))
            self.assertEqual(self.frame(*interrupt)[:2], (0., 0.), interrupt)

    def test_switching_hands_keeps_driving_after_a_new_hold(self):
        self.hold('FORWARD', 'NONE')
        self.assertEqual(self.frame('FORWARD', 'NONE')[:2], (.5, .5))
        self.assertEqual(self.frame('NONE', 'FORWARD')[:2], (.5, .5))   # same command, other hand
        self.assertEqual(self.control.active, (1,))

    def test_changing_command_restarts_the_hold(self):
        self.hold('FORWARD')
        self.assertEqual(self.frame('RIGHT')[:2], (0., 0.))
        self.assertNotEqual(self.hold('RIGHT')[-1][:2], (0., 0.))

    def test_stale_camera_stops(self):
        self.hold('FORWARD')
        self.t += .05
        stale = Hand(self.t - .3, 'FORWARD')
        self.assertEqual(self.control.update([stale], self.t)[:2], (0., 0.))

    def test_frozen_frame_cannot_finish_a_hold(self):
        self.t += .05
        hand = Hand(self.t, 'FORWARD')
        for _ in range(5):
            self.assertEqual(self.control.update([hand], self.t + .01)[:2], (0., 0.))

    def test_grip_fires_once_per_hold_and_never_moves(self):
        events = [e for o in self.hold('GRIP_CLOSE', seconds=1.5) for e in o[2]]
        self.assertEqual(events, [('grip', 'close')])
        self.assertTrue(all(o[:2] == (0., 0.) for o in self.hold('GRIP_CLOSE')))
        self.hold('NONE', seconds=.1)
        events = [e for o in self.hold('GRIP_OPEN', seconds=GRIP_HOLD_S + .1) for e in o[2]]
        self.assertEqual(events, [('grip', 'open')])

    def test_short_grip_flash_does_nothing(self):
        out = self.hold('GRIP_CLOSE', seconds=GRIP_HOLD_S - .1) + self.hold('NONE', seconds=.2)
        self.assertFalse(any(o[2] for o in out))

    def test_pose_held_while_paused_must_be_held_again_after_start(self):
        self.control.pause()
        self.hold('GRIP_CLOSE', seconds=1.)
        self.control.start()
        self.assertEqual(self.frame('GRIP_CLOSE'), (0., 0., []))
        events = [e for o in self.hold('GRIP_CLOSE', seconds=GRIP_HOLD_S + .1) for e in o[2]]
        self.assertEqual(events, [('grip', 'close')])

    def test_pose_held_through_start_drives_after_a_new_hold(self):
        self.control.pause()
        self.hold('FORWARD')
        self.control.start()
        self.assertEqual(self.frame('FORWARD')[:2], (0., 0.))
        self.assertEqual(self.hold('FORWARD', seconds=MOVE_HOLD_S + .05)[-1][:2], (.5, .5))

    def test_speed_keys_step_clamp_and_apply_at_once(self):
        self.hold('FORWARD')
        self.assertEqual(self.control.adjust_speed(+1), .6)
        self.assertEqual(self.frame('FORWARD')[:2], (.6, .6))
        for _ in range(10):
            self.control.adjust_speed(+1)
        self.assertEqual(self.control.speed, 1.)
        for _ in range(20):
            self.control.adjust_speed(-1)
        self.assertEqual(self.control.speed, .1)

    def test_invalid_settings_rejected(self):
        for speed in (0., .05, 1.1, math.nan):
            with self.assertRaises(ValueError):
                GestureDrive(speed)
        with self.assertRaises(ValueError):
            GestureDrive(.3, timeout=.5)

    def test_duty_mapping(self):
        self.assertEqual(duty(0.), 0.)
        self.assertAlmostEqual(duty(1., .65), 1.)
        self.assertAlmostEqual(duty(.5, .6), .8)


class FakeLink:
    def __init__(self):
        self.session = '0123456789ab'
        self.status = None
        self.status_at = 0.
        self.packets = []
        self.closed = False
        self.status_extra = {}

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
        self.control = GestureDrive(.5)
        self.link = FakeLink()
        self.session = GestureSession(self.control, self.link)
        self.t = 10.

    def status(self, session=None, state='RUNNING'):
        self.link.status = {'state': state, 'session': session or self.link.session, **self.link.status_extra}
        self.link.status_at = self.t

    def frame(self, pose='FORWARD', dt=.05, status=True):
        self.t += dt
        if status:
            self.status()
        hands = [] if pose is None else [Hand(self.t, pose)]
        return self.session.tick(hands, self.t)

    def hold(self, pose='FORWARD', n=10):
        return [self.frame(pose) for _ in range(n)]

    def running(self):
        self.session.start(self.t)
        self.frame('NONE')
        self.assertTrue(self.control.enabled)

    def test_preview_needs_no_link_and_starts_locally(self):
        s = GestureSession(self.control)
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
        self.session.tick([], self.t)
        self.assertFalse(self.control.enabled)
        self.t += .1
        self.status()
        self.session.tick([], self.t)
        self.assertTrue(self.control.enabled)

    def test_start_retried_then_times_out_stopped(self):
        self.session.start(self.t)
        self.session.tick([], self.t+.3)
        self.assertEqual(sum(c == 'start' for c, _ in self.link.packets), 2)
        self.session.tick([], self.t+1.6)
        self.assertFalse(self.control.enabled)
        self.assertIsNone(self.session.pending_start)
        self.assertEqual(self.link.packets[-1][0], 'stop')

    def test_drives_after_running_and_stops_on_status_loss(self):
        self.running()
        self.assertEqual(self.hold()[-1][:2], (.5, .5))
        self.assertIn(('drive', {'l': .5, 'r': .5}), self.link.packets)
        self.frame(dt=.7, status=False)
        self.assertFalse(self.control.enabled)
        self.assertEqual(self.link.packets[-1][0], 'stop')
        self.hold()
        self.assertFalse(self.control.enabled)       # no auto-resume

    def test_session_change_stops_control(self):
        self.running()
        self.hold()
        self.t += .05
        self.status(session='other')
        self.session.tick([Hand(self.t, 'FORWARD')], self.t)
        self.assertFalse(self.control.enabled)

    def test_stop_packet_bypasses_20_hz_interval(self):
        self.running()
        self.hold()
        self.link.packets.clear()
        self.frame('NONE', dt=.001)
        self.assertIn(('drive', {'l': 0., 'r': 0.}), self.link.packets)

    def test_grip_retried_three_times_with_zero_wheels_first(self):
        self.running()
        self.link.packets.clear()
        self.hold('GRIP_CLOSE', n=20)
        grips = [(i, p) for i, p in enumerate(self.link.packets) if p[0] == 'grip']
        self.assertEqual(len(grips), 3)
        for i, packet in grips:
            self.assertEqual(packet, ('grip', {'p': 'close'}))
            self.assertEqual(self.link.packets[i-1], ('drive', {'l': 0., 'r': 0.}))
        self.assertFalse(any(c == 'drive' and (f['l'] or f['r']) for c, f in self.link.packets))

    def test_stop_key_cancels_queued_grip_copies(self):
        self.running()
        for _ in range(20):
            if self.frame('GRIP_OPEN')[2]:
                break
        self.assertIsNotNone(self.session.grip_retry)
        self.session.stop()
        self.link.packets.clear()
        self.frame('GRIP_OPEN')
        self.assertFalse(any(c == 'grip' for c, _ in self.link.packets))

    def test_close_stops_and_closes_socket(self):
        self.running()
        self.session.close()
        self.assertTrue(self.link.closed)
        self.assertFalse(self.control.enabled)
        self.assertEqual(self.link.packets[-1], ('stop', {}))

    def test_min_duty_sent_with_every_drive_packet(self):
        self.session = GestureSession(self.control, self.link, min_duty=.65)
        self.link.status_extra = {'min_duty': .65}
        self.running()
        self.hold()
        self.frame('NONE')
        drives = [f for c, f in self.link.packets if c == 'drive']
        self.assertTrue(drives)
        self.assertTrue(all(f.get('m') == .65 for f in drives))
        self.assertIn(('drive', {'l': .5, 'r': .5, 'm': .65}), self.link.packets)

    def test_default_sends_no_floor(self):
        self.running()
        self.hold()
        self.assertFalse(any('m' in f for c, f in self.link.packets if c == 'drive'))

    def test_min_duty_with_firmware_that_does_not_report_it_stops(self):
        self.session = GestureSession(self.control, self.link, min_duty=.65)
        self.session.start(self.t)
        self.hold()
        self.assertFalse(self.control.enabled)
        self.assertIn('min_duty', self.session.message)
        self.assertFalse(any(c == 'drive' and (f['l'] or f['r']) for c, f in self.link.packets))

    def test_invalid_min_duty_rejected(self):
        for floor in (-.1, 1.1, math.nan):
            with self.assertRaises(ValueError):
                GestureSession(self.control, self.link, min_duty=floor)


@unittest.skipUnless(importlib.util.find_spec('cv2'), 'Install requirements for UI smoke tests')
class AppTests(unittest.TestCase):
    def run_app(self, argv, keys):
        import gesture_control
        with patch('gesture_control.CameraWorker', side_effect=AssertionError('demo opened camera')), \
             patch('gesture_control.RobotLink', side_effect=AssertionError('preview opened socket')), \
             patch('cv2.namedWindow'), patch('cv2.imshow') as show, patch('cv2.destroyAllWindows'), \
             patch('cv2.getWindowProperty', return_value=1), patch('cv2.waitKey', side_effect=keys):
            code = gesture_control.main(argv)
        return code, show

    def test_demo_runs_every_pose_without_camera_or_socket(self):
        keys = [ord('g')] + [ord(str(k)) for k in range(1, 9)] + [ord('9'), ord('0'), ord('x'), 27]
        code, show = self.run_app(['--demo'], keys)
        self.assertEqual(code, 0)
        self.assertEqual(show.call_count, len(keys))
        self.assertEqual(show.call_args.args[1].shape, (720, 1280, 3))

    def test_plus_minus_keys_change_speed(self):
        import gesture_control
        with patch.object(gesture_control.GestureDrive, 'adjust_speed', autospec=True,
                          side_effect=gesture_control.GestureDrive.adjust_speed) as adjust:
            self.run_app(['--demo'], [ord('+'), ord('='), ord('-'), 27])
        self.assertEqual([c.args[1] for c in adjust.call_args_list], [1, 1, -1])

    def test_bad_arguments_rejected_before_socket(self):
        import gesture_control
        for argv in (['--demo', '--live', '--robot', '127.0.0.1'], ['--live'],
                     ['--demo', '--min-duty', '1.5'], ['--demo', '--speed', '0']):
            with patch('gesture_control.RobotLink', side_effect=AssertionError('socket opened')), \
                 patch('sys.stderr'), self.assertRaises(SystemExit) as error:
                gesture_control.main(argv)
            self.assertEqual(error.exception.code, 2, argv)

    def test_missing_model_is_a_clear_error(self):
        import gesture_control
        with patch('gesture_control.newest_model', return_value=None), patch('sys.stderr'), \
             self.assertRaises(SystemExit) as error:
            gesture_control.main([])
        self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
