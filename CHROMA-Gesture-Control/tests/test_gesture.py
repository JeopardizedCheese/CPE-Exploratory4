import json
import math
import socket
import time
import unittest

from gesture_logic import GestureControl, Hand, RobotLink, classify_landmarks


class GestureTests(unittest.TestCase):
    def setUp(self):
        self.c = GestureControl()
        self.c.start()
        self.t = 10.

    def frame(self, pose='OPEN', x=.5, y=.5, dt=.1):
        self.t += dt
        return self.c.update(Hand(self.t, pose, x, y), self.t)

    def hold(self, pose='OPEN', n=8, x=.5, y=.5):
        results = [self.frame(pose, x, y) for _ in range(n)]
        return [r[2] for r in results if r[2]]

    def ready(self):
        self.hold()
        self.assertTrue(self.c.ready)

    def test_preview_requires_explicit_start(self):
        self.c.pause()
        self.hold()
        self.assertEqual(self.frame(x=.8), (0., 0., None))

    def test_neutral_required_before_motion(self):
        self.hold(x=.8)
        self.assertFalse(self.c.ready)
        self.ready()
        self.assertEqual(self.frame(y=.2), (.25, .25, None))

    def test_directions(self):
        self.ready()
        self.assertEqual(self.frame(y=.8)[:2], (-.25, -.25))
        self.assertEqual(self.frame(x=.8)[:2], (.15, -.15))
        self.assertEqual(self.frame(x=.2)[:2], (-.15, .15))
        self.assertEqual(self.frame()[:2], (0., 0.))

    def test_missing_hand_stops_and_requires_neutral(self):
        self.ready()
        self.frame(y=.2)
        self.assertEqual(self.c.update(None, self.t), (0., 0., None))
        self.assertEqual(self.frame(y=.2), (0., 0., None))
        self.ready()

    def test_frozen_result_cannot_sustain_drive(self):
        self.ready()
        self.frame(y=.2)
        old = Hand(self.t, 'OPEN', .5, .2)
        self.assertEqual(self.c.update(old, self.t+.21), (0., 0., None))
        self.assertFalse(self.c.ready)

    def test_repeated_result_cannot_complete_dwell(self):
        first = Hand(self.t, 'OPEN')
        for dt in (.01, .05, .1, .15, .19):
            self.c.update(first, self.t+dt)
        self.assertFalse(self.c.ready)

    def test_unknown_and_fist_stop(self):
        for pose in ('UNKNOWN', 'FIST', 'ONE'):
            self.ready()
            self.assertEqual(self.frame(pose), (0., 0., None))
            self.assertFalse(self.c.ready)

    def test_bad_timestamp_coordinates_rejected(self):
        for hand in (Hand(self.t+1, 'OPEN'), Hand(self.t, 'OPEN', math.nan),
                     Hand(self.t, 'OPEN', 2), Hand(math.inf, 'OPEN')):
            self.assertEqual(self.c.update(hand, self.t), (0., 0., None))

    def test_frame_gap_disarms_even_if_new_frame_is_fresh(self):
        self.ready()
        self.assertEqual(self.frame(y=.2, dt=.5), (0., 0., None))
        self.assertFalse(self.c.ready)

    def test_mode_switch_only_once_during_held_v(self):
        self.ready()
        self.hold('V', n=25)
        self.assertEqual(self.c.mode, 'ARM')
        self.assertFalse(self.c.ready)
        self.ready()
        self.hold('V', n=12)
        self.assertEqual(self.c.mode, 'DRIVE')

    def test_arm_actions_one_shot_with_neutral_between(self):
        self.hold('V', n=12)
        for pose, expected in [('ONE', ('grip', 'open')), ('THREE', ('grip', 'close'))]:
            self.ready()
            self.assertEqual(self.hold(pose, n=20), [expected])
            self.assertEqual(self.hold(pose, n=10), [])

    def test_arm_dwell_interrupted_by_hand_loss(self):
        self.hold('V', n=12)
        self.ready()
        self.hold('ONE', n=3)
        self.c.update(None, self.t)
        self.assertEqual(self.hold('ONE', n=15), [])

    def test_arm_never_drives(self):
        self.hold('V', n=12)
        self.ready()
        self.assertEqual(self.frame(x=.8), (0., 0., None))

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
        self.assertEqual(self.c.update(Hand(self.t-.05, 'OPEN', .5, .2), self.t),
                         (0., 0., None))
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
                receiver.sendto(b'{"state":"RUNNING"}', address)
                deadline = time.monotonic()+1
                while link.status is None and time.monotonic() < deadline:
                    link.poll()
                    time.sleep(.001)
                self.assertTrue(link.running(time.monotonic()))
                self.assertFalse(link.running(link.status_at+.61))
                link.status = {'state': 'ESTOP'}
                self.assertFalse(link.running(time.monotonic()))
            finally:
                link.close()


if __name__ == '__main__':
    unittest.main()