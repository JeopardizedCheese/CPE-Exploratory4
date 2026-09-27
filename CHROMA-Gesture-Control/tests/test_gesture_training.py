import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from gesture_collect import Recording
from gesture_logic import GestureControl, Hand
from gesture_model import FEATURE_COUNT, FEATURE_VERSION, LABELS, GestureModel, gated_labels, landmark_features
from gesture_train import load_recordings, metrics, split_sessions, train


def example_landmarks():
    rng = np.random.default_rng(3)
    points = rng.uniform(.3, .7, (21, 3))
    points[:, 2] *= .1
    points[0] = [.5, .8, 0]
    points[9] = [.5, .5, -.01]
    return points


class FeatureTests(unittest.TestCase):
    def test_translation_and_scale_invariant_but_orientation_preserved(self):
        p = example_landmarks()
        expected = landmark_features(p, 640, 480)
        np.testing.assert_allclose(expected, landmark_features(p*.7+[.1, .1, .1], 640, 480))
        upside_down = p.copy()
        upside_down[:, :2] = 1-upside_down[:, :2]
        self.assertFalse(np.allclose(expected, landmark_features(upside_down, 640, 480)))

    def test_invalid_or_tiny_hand_rejected(self):
        self.assertIsNone(landmark_features(np.zeros((21, 3)), 640, 480))
        self.assertIsNone(landmark_features(np.ones((20, 3)), 640, 480))
        self.assertIsNone(landmark_features(np.full((21, 3), np.nan), 640, 480))

    def test_low_confidence_and_ambiguous_predictions_rejected(self):
        classes = np.array(LABELS)
        p = np.full((3, 8), .01)
        p[0, 0] = .6  # below threshold
        p[1, 0], p[1, 1] = .51, .48  # small margin
        p[2, 0] = .93
        self.assertEqual(gated_labels(p, classes, .9, .2).tolist(), ['UNKNOWN', 'UNKNOWN', 'OPEN'])


class CollectionTests(unittest.TestCase):
    def hand(self, t):
        return Hand(t, 'OPEN', features=tuple(np.zeros(FEATURE_COUNT)))

    def test_countdown_duplicate_stale_frames_and_sample_limit(self):
        rec = Recording('OPEN', 10., target=2)
        self.assertFalse(rec.add(self.hand(11.), 11.))
        self.assertTrue(rec.add(self.hand(12.1), 12.1))
        self.assertFalse(rec.add(self.hand(12.1), 12.15))
        self.assertFalse(rec.add(self.hand(12.3), 12.7))
        self.assertFalse(rec.add(None, 12.7))
        self.assertTrue(rec.add(self.hand(12.8), 12.8))
        self.assertFalse(rec.add(self.hand(13.1), 13.1))
        self.assertEqual(len(rec.samples), 2)

    def test_roundtrip_uses_human_label_not_rule_prediction(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = Recording('THREE', 0.)
            rec.add(self.hand(2.1), 2.1)  # detector says OPEN, human label is THREE
            path = rec.save(Path(tmp)/'session-a', 'session-a')
            x, y, groups, files = load_recordings(tmp)
            self.assertEqual(x.shape, (1, FEATURE_COUNT))
            self.assertEqual(y.tolist(), ['THREE'])
            self.assertEqual(groups.tolist(), ['session-a'])
            self.assertTrue(path.exists())
            self.assertEqual(len(files), 1)

    def test_duplicate_file_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = Recording('OPEN', 0.)
            rec.add(self.hand(2.1), 2.1)
            path = rec.save(Path(tmp)/'session-a', 'session-a')
            path.with_name('duplicate.npz').write_bytes(path.read_bytes())
            with self.assertRaisesRegex(ValueError, 'Duplicate'):
                load_recordings(tmp)

    def test_collector_ui_save_undo_and_resume_without_real_camera(self):
        import cv2
        import threading
        import gesture_collect as app
        clock = [100.]

        class Worker:
            def __init__(self, *args):
                self.end = threading.Event()
                self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
            def start(self):
                pass
            def join(self, **kwargs):
                pass
            def snapshot(self):
                return self.frame, Hand(clock[0], 'OPEN', features=tuple(np.zeros(FEATURE_COUNT))), ''

        def run(argv, keys):
            sequence = iter(keys)
            def keypress(_):
                clock[0] += .21
                return next(sequence)
            with patch.object(app, 'CameraWorker', Worker), patch.object(app.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(cv2, 'imshow'), patch.object(cv2, 'getWindowProperty', return_value=1), \
                    patch.object(cv2, 'waitKey', side_effect=keypress), patch.object(cv2, 'destroyAllWindows'), \
                    patch('sys.argv', argv):
                app.main()

        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)/'placeholder.task'
            model.write_bytes(b'not read by fake worker')
            directory = Path(tmp)/'data'
            argv = ['gesture_collect.py', '--data', str(directory), '--model', str(model), '--samples', '30']
            run(argv, [ord('5'), ord('r')]+[-1]*50+[ord('u'), ord('r')]+[-1]*50+[27])
            x, y, groups, inventory = load_recordings(directory)
            self.assertEqual(x.shape, (30, FEATURE_COUNT))
            self.assertEqual(set(y), {'THREE'})
            self.assertEqual(len(inventory), 1)
            session = str(groups[0])
            self.assertEqual(len(list((directory/session/'discarded').glob('*.npz'))), 1)
            run(argv+['--session', session], [27])
            self.assertEqual(len(list(directory.iterdir())), 1)


class SplitTests(unittest.TestCase):
    def data(self, sessions=5):
        y = np.tile(np.repeat(LABELS, 30), sessions)
        groups = np.repeat([f's{i}' for i in range(sessions)], 30*len(LABELS))
        return y, groups

    def test_no_session_leakage_and_every_label_present(self):
        y, groups = self.data()
        split = split_sessions(y, groups)
        names = list(split)
        for name in names:
            self.assertEqual(set(y[split[name]]), set(LABELS))
            for other in names:
                if name != other:
                    self.assertTrue(set(groups[split[name]]).isdisjoint(groups[split[other]]))
        self.assertEqual(sum(len(v) for v in split.values()), len(y))

    def test_insufficient_or_incomplete_sessions_fail(self):
        with self.assertRaisesRegex(ValueError, 'at least 5'):
            split_sessions(*self.data(4))
        y, groups = self.data()
        keep = ~((groups == 's0') & (y == 'UNKNOWN'))
        with self.assertRaisesRegex(ValueError, 'Missing/short'):
            split_sessions(y[keep], groups[keep])

    def test_report_counts_false_commands_not_only_accuracy(self):
        report = metrics(np.array(['UNKNOWN', 'OPEN', 'FIST']),
                         np.array(['ONE', 'UNKNOWN', 'FIST']))
        self.assertEqual(report['wrong_active_commands'], 1)
        self.assertEqual(report['unknown_to_active_rate'], 1.)
        self.assertEqual(report['correct_active_recall'], 0.)


class TrainingIntegrationTests(unittest.TestCase):
    def test_live_loop_accepts_fresh_status_after_poll(self):
        import cv2
        import threading
        import gesture_control as app
        clock = [100.]
        packets = []

        class Link:
            def __init__(self, *args):
                self.started, self.received = False, 0.
            def poll(self):
                clock[0] += .001
                self.received = clock[0]
            def running(self, now):
                return self.started and 0 <= now-self.received < .6
            def send(self, cmd, **fields):
                packets.append((cmd, fields))
                if cmd == 'start':
                    self.started = True
            def close(self):
                pass

        class Worker:
            def __init__(self, *args):
                self.end = threading.Event()
                self.count = 0
            def start(self):
                pass
            def join(self, **kwargs):
                pass
            def snapshot(self):
                self.count += 1
                return np.zeros((480, 640, 3), dtype=np.uint8), Hand(
                    clock[0], 'OPEN', y=.5 if self.count < 12 else .2), ''

        keys = iter([ord('g')]+[-1]*18+[27])
        def keypress(_):
            clock[0] += .1
            return next(keys)
        with tempfile.TemporaryDirectory() as tmp:
            model = Path(tmp)/'placeholder.task'
            model.write_bytes(b'fake worker does not load this')
            with patch.object(app, 'CameraWorker', Worker), patch.object(app, 'RobotLink', Link), \
                    patch.object(app.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(cv2, 'imshow'), patch.object(cv2, 'getWindowProperty', return_value=1), \
                    patch.object(cv2, 'waitKey', side_effect=keypress), patch.object(cv2, 'destroyAllWindows'), \
                    patch('sys.argv', ['gesture_control.py', '--model', str(model), '--live', '--robot', '127.0.0.1']):
                app.main()
        self.assertTrue(any(cmd == 'drive' and fields.get('l', 0) > 0 for cmd, fields in packets))
        self.assertEqual(packets[-1][0], 'stop')

    def test_train_export_reload_report_and_control_use(self):
        # Synthetic separable features test plumbing ONLY, not real hand accuracy.
        rng = np.random.default_rng(5)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for s in range(5):
                directory = root/'data'/f's{s}'
                directory.mkdir(parents=True)
                for i, label in enumerate(LABELS):
                    x = rng.normal(0, .04, (30, FEATURE_COUNT))
                    x[:, i] += 2.
                    np.savez_compressed(directory/f'{label}.npz', X=x, label=np.array(label),
                                        session=np.array(f's{s}'), feature_version=np.array(FEATURE_VERSION))
            output = root/'model.npz'
            report = train(root/'data', output, iterations=120)
            model = GestureModel(output)
            self.assertTrue(output.with_suffix('.report.json').exists())
            self.assertFalse(report['hardware_validated'])
            self.assertEqual(len(report['splits']['train']['sessions']), 3)
            feature = np.zeros(FEATURE_COUNT)
            feature[0] = 2.
            label, score, candidate = model.predict(feature)
            self.assertEqual(label, 'OPEN')
            self.assertEqual(candidate, 'OPEN')
            control = GestureControl()
            control.start()
            for i in range(10):
                t = 10+i*.1
                control.update(Hand(t, label, score=score), t)
            self.assertTrue(control.ready)
            t += .1
            self.assertEqual(control.update(Hand(t, label, y=.2), t)[:2], (.25, .25))
            t += .1
            rejected = gated_labels(np.ones((1, 8))/8, model.classes, model.threshold, model.margin)[0]
            self.assertEqual(control.update(Hand(t, rejected, y=.2), t)[:2], (0., 0.))
            self.assertFalse(control.ready)
            with self.assertRaisesRegex(ValueError, 'already exists'):
                train(root/'data', output)
            # Corrupt metadata must not load as a model or fall back to rules.
            with np.load(output, allow_pickle=False) as data:
                arrays = {name: data[name] for name in data.files}
            meta = json.loads(str(arrays['metadata'].item()))
            meta['feature_version'] = 'other'
            arrays['metadata'] = np.array(json.dumps(meta))
            np.savez(root/'bad.npz', **arrays)
            with self.assertRaisesRegex(ValueError, 'feature version'):
                GestureModel(root/'bad.npz')


if __name__ == '__main__':
    unittest.main()
