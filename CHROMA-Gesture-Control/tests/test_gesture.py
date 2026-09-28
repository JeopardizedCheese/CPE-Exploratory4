import json
import math
import socket
import time
import unittest

from gesture_logic import CANNED, GestureControl, Hand, RobotLink, canned_pose, classify_landmarks, duty

DRIVE_X, CMD_X = .75, .25  # centres of the drive (screen-right) and command halves


class GestureTests(unittest.TestCase):
    def setUp(self):
        self.c = GestureControl(gears=(.2, .5))
        self.c.start()
        self.t = 10.

    def frame(self, pose='OPEN', x=DRIVE_X, y=.5, dt=.1, command=None):
        """One tick with a drive hand (pose=None: absent) and optional command hand."""
        self.t += dt
        hands = []
        if pose is not None:
            hands.append(Hand(self.t, pose, x, y))
        if command is not None:
            hands.append(Hand(self.t, command, CMD_X, .5))
        return self.c.update(hands, self.t)

    def hold(self, pose='OPEN', n=8, x=DRIVE_X, y=.5, command=None):
        results = [self.frame(pose, x, y, command=command) for _ in range(n)]
        return [e for r in results for e in r[2]]

    def ready(self):
        self.hold()
        self.assertTrue(self.c.ready)

    def command(self, pose, n=12):
        return self.hold(None, n=n, command=pose)

    def test_preview_requires_explicit_start(self):
        self.c.pause()
        self.hold()
        self.assertEqual(self.frame(x=.95), (0., 0., []))

    def test_neutral_required_before_motion(self):
        self.hold(x=.95)
        self.assertFalse(self.c.ready)
        self.ready()
        self.assertEqual(self.frame(y=.2), (.2, .2, []))

    def test_directions(self):
        self.ready()
        self.assertEqual(self.frame(y=.8)[:2], (-.2, -.2))
        r, l = self.frame(x=.95)[:2], self.frame(x=.55)[:2]
        self.assertAlmostEqual(r[0], .12)
        self.assertAlmostEqual(r[1], -.12)
        self.assertAlmostEqual(l[0], -.12)
        self.assertEqual(self.frame()[:2], (0., 0.))

    def test_drive_hand_crossing_midline_stops(self):
        self.ready()
        self.assertEqual(self.frame(x=.45)[:2], (0., 0.))
        self.assertFalse(self.c.ready)

    def test_two_hands_on_one_side_stop_everything(self):
        self.ready()
        self.t += .1
        hands = [Hand(self.t, 'OPEN', .9, .2), Hand(self.t, 'OPEN', .8, .2)]
        self.assertEqual(self.c.update(hands, self.t), (0., 0., []))
        self.assertFalse(self.c.ready)

    def test_missing_hand_stops_and_requires_neutral(self):
        self.ready()
        self.frame(y=.2)
        self.assertEqual(self.c.update([], self.t), (0., 0., []))
        self.assertEqual(self.frame(y=.2), (0., 0., []))
        self.ready()

    def test_frozen_result_cannot_sustain_drive(self):
        self.ready()
        self.frame(y=.2)
        old = Hand(self.t, 'OPEN', DRIVE_X, .2)
        self.assertEqual(self.c.update([old], self.t+.21), (0., 0., []))
        self.assertFalse(self.c.ready)

    def test_repeated_result_cannot_complete_dwell(self):
        first = Hand(self.t, 'OPEN', DRIVE_X)
        for dt in (.01, .05, .1, .15, .19):
            self.c.update([first], self.t+dt)
        self.assertFalse(self.c.ready)

    def test_unknown_and_fist_stop(self):
        for pose in ('UNKNOWN', 'FIST', 'ONE'):
            self.ready()
            self.assertEqual(self.frame(pose), (0., 0., []))
            self.assertFalse(self.c.ready)

    def test_bad_timestamp_coordinates_rejected(self):
        for hand in (Hand(self.t+1, 'OPEN', DRIVE_X), Hand(self.t, 'OPEN', math.nan),
                     Hand(self.t, 'OPEN', 2), Hand(math.inf, 'OPEN', DRIVE_X)):
            self.assertEqual(self.c.update([hand], self.t), (0., 0., []))

    def test_frame_gap_disarms_even_if_new_frame_is_fresh(self):
        self.ready()
        self.assertEqual(self.frame(y=.2, dt=.5), (0., 0., []))
        self.assertFalse(self.c.ready)

    def test_command_needs_open_rearm_between_commands(self):
        self.assertEqual(self.command('ONE'), [])            # never armed
        self.command('OPEN', n=5)
        self.assertEqual(self.command('ONE'), [('grip', 'open')])
        self.assertEqual(self.command('THREE'), [])          # no OPEN in between
        self.command('OPEN', n=5)
        self.assertEqual(self.command('THREE'), [('grip', 'close')])

    def test_command_dwell_interrupted_by_hand_loss(self):
        self.command('OPEN', n=5)
        self.command('ONE', n=3)
        self.frame(None)
        self.assertEqual(self.command('ONE', n=15), [])

    def test_gears_shift_and_clamp(self):
        self.command('OPEN', n=5)
        self.assertEqual(self.command('THUMB_UP'), [('gear', 2)])
        self.command('OPEN', n=5)
        self.assertEqual(self.command('THUMB_UP'), [('gear', 2)])  # already top gear
        self.ready()
        self.assertEqual(self.frame(y=.2)[:2], (.5, .5))
        self.command('OPEN', n=5)
        self.assertEqual(self.command('THUMB_DOWN'), [('gear', 1)])
        self.assertEqual(self.c.speed, .2)

    def test_both_hands_at_once_drive_and_grip(self):
        self.hold(n=8, command='OPEN')
        self.assertTrue(self.c.ready and self.c.armed)
        results = [self.frame(y=.2, command='THREE') for _ in range(6)]
        self.assertTrue(all(r[:2] == (.2, .2) for r in results))
        self.assertEqual([e for r in results for e in r[2]], [('grip', 'close')])

    def test_paused_only_accepts_start_and_stop(self):
        self.c.pause()
        self.command('OPEN', n=5)
        self.assertEqual(self.command('ONE'), [])
        self.assertEqual(self.command('V', n=12), [('start', None)])
        self.assertEqual(self.command('FIST', n=8), [('stop', None)])

    def test_fist_stop_needs_no_rearm_and_fires_once(self):
        self.assertEqual(self.command('FIST', n=8), [('stop', None)])
        self.c.pause()  # what gesture_control does on stop
        self.assertEqual(self.command('FIST', n=20), [])

    def test_swap_hands(self):
        c = GestureControl(gears=(.2,), swap=True)
        c.start()
        t = 10.
        for _ in range(8):
            t += .1
            c.update([Hand(t, 'OPEN', CMD_X, .5)], t)
        self.assertTrue(c.ready)
        t += .1
        self.assertEqual(c.update([Hand(t, 'OPEN', CMD_X, .2)], t)[:2], (.2, .2))

    def test_mediapipe_canned_gestures_map_to_commands(self):
        self.assertEqual(canned_pose('Pointing_Up', .9, .7), 'ONE')
        self.assertEqual(canned_pose('Pointing_Up', .6, .7), 'UNKNOWN')
        self.assertEqual(canned_pose('Something_New', .99, .7), 'UNKNOWN')
        self.assertEqual(set(CANNED.values()) - {'UNKNOWN', 'OPEN'},
                         {'FIST', 'V', 'ONE', 'THUMB_UP', 'THUMB_DOWN', 'ILOVEYOU'})
        self.command('OPEN', n=5)
        self.assertEqual(self.command('ILOVEYOU'), [('grip', 'close')])

    def test_duty_matches_firmware_mapping(self):
        self.assertEqual(duty(0), 0.)
        self.assertAlmostEqual(duty(.18), .7622)
        self.assertAlmostEqual(duty(-1), 1.)

    def test_classifier_rejects_invalid_landmarks(self):
        self.assertEqual(classify_landmarks([]), 'UNKNOWN')
        self.assertEqual(classify_landmarks([(math.nan, 0)]*21), 'UNKNOWN')

    def test_classifier_distinguishes_supported_finger_patterns(self):
        # Synthetic upright palm with fingers extended or folded toward wrist.
        def points(extended):
            p = [(.5, .9)]*21
            p[1:5] = [(.38, .78), (.30, .70), (.24, .65), (.20, .60)]
            for m, x, opened in zip((5, 9, 13, 17), (.3, .43, .56, .7), extended):
                ys = (.65, .45, .30, .15) if opened else (.65, .55, .67, .77)
                p[m:m+4] = [(x, y) for y in ys]
            return p
        for flags, expected in [((1, 1, 1, 1), 'OPEN'), ((1, 1, 0, 0), 'V'),
                                ((1, 0, 0, 0), 'ONE'), ((1, 1, 1, 0), 'THREE')]:
            self.assertEqual(classify_landmarks(points(flags)), expected)
        p = points((0, 0, 0, 0))
        p[2:5] = [(.15, .65), (.15, .4), (.15, .15)]
        self.assertEqual(classify_landmarks(p), 'THUMB_UP')
        p[2:5] = [(.15, .65), (.15, .9), (.15, 1.15)]
        self.assertEqual(classify_landmarks(p), 'THUMB_DOWN')

    def test_backward_timestamp_disarms(self):
        self.ready()
        self.assertEqual(self.c.update([Hand(self.t-.05, 'OPEN', DRIVE_X, .2)], self.t),
                         (0., 0., []))
        self.assertFalse(self.c.ready)


class ProtocolTests(unittest.TestCase):
    def test_v3_transport_and_fresh_status_gate(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.settimeout(1)
            link = RobotLink('127.0.0.1', receiver.getsockname()[1])
            try:
                self.assertFalse(link.running(time.monotonic()))
                packets = []
                for cmd, fields in [('start', {}), ('drive', {'l': .25, 'r': .25}),
                                    ('grip', {'p': 'close'}), ('stop', {})]:
                    link.send(cmd, **fields)
                    data, address = receiver.recvfrom(2048)
                    packets.append(json.loads(data))
                self.assertEqual([p['q'] for p in packets], [1, 2, 3, 4])
                self.assertEqual({p['v'] for p in packets}, {3})
                self.assertEqual(len({p['s'] for p in packets}), 1)
                self.assertEqual(len(packets[0]['s']), 12)
                self.assertEqual(packets[1]['l'], .25)
                receiver.sendto(b'{"state":"DONE"}', address)  # not a robot_ctrl state
                receiver.sendto(b'{"state":"RUNNING"}', address)
                deadline = time.monotonic()+1
                while link.status is None and time.monotonic() < deadline:
                    link.poll()
                    time.sleep(.001)
                self.assertTrue(link.running(time.monotonic()))
                self.assertFalse(link.running(link.status_at+.61))
                link.status = {'state': 'IDLE'}
                self.assertFalse(link.running(time.monotonic()))
            finally:
                link.close()


if __name__ == '__main__':
    unittest.main()