import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from gesture_collect import (MIN_SAMPLES, Recording, next_missing, readiness, render_collect, scan)
from gesture_logic import Hand
from gesture_model import (FEATURE_COUNT, FEATURE_VERSION, HANDS, LABEL_SET, LABELS, SPLITS,
                           GestureModel, gated_labels, landmark_features, newest_model)
import gesture_train
from gesture_train import check_data, choose_gate, load_recordings, metrics, train

HAS_SKLEARN = importlib.util.find_spec('sklearn') is not None


def example_landmarks():
    rng = np.random.default_rng(3)
    points = rng.uniform(.3, .7, (21, 3))
    points[:, 2] *= .1
    points[0] = [.5, .8, 0]
    points[9] = [.5, .5, -.01]
    return points


CENTRES = np.random.default_rng(0).normal(0, 1, (len(LABELS), FEATURE_COUNT))


def make_session(root, split, n=40, noise=.25, seed=1, labels=LABELS):
    """A session on disk as gesture_collect.py writes it; features cluster per label and hand."""
    rng = np.random.default_rng(seed)
    session = f'{split}-{seed}'
    directory = Path(root)/session
    directory.mkdir(parents=True)
    (directory/'session.json').write_text(json.dumps(
        {'session': session, 'split': split, 'feature_version': FEATURE_VERSION, 'label_set': LABEL_SET}))
    for label in labels:
        for hand in HANDS:
            rec = Recording(label, 0., n, hand)
            centre = CENTRES[LABELS.index(label)] + (.3 if hand == 'L' else 0.)
            rec.samples = [centre + rng.normal(0, noise, FEATURE_COUNT) for _ in range(n)]
            rec.timestamps = list(range(n))
            rec.save(directory, session, split)
    return directory


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

    def test_low_confidence_and_ambiguous_predictions_become_none(self):
        classes = np.array(LABELS)
        p = np.zeros((3, len(LABELS)))
        p[0, 1] = .85; p[0, 2] = .15                 # below threshold
        p[1, 1] = .55; p[1, 2] = .45                 # too close to the second
        p[2, 2] = .97; p[2, 1] = .03
        self.assertEqual(gated_labels(p, classes, .9, .2).tolist(), ['NONE', 'NONE', 'FORWARD'])


class CollectionTests(unittest.TestCase):
    def test_countdown_duplicate_stale_frames_and_sample_limit(self):
        rec = Recording('FORWARD', 0., target=2)
        f = tuple(np.ones(FEATURE_COUNT))
        self.assertFalse(rec.add(Hand(1., 'NONE', features=f), 1.))            # countdown
        self.assertTrue(rec.add(Hand(2.1, 'NONE', features=f), 2.1))
        self.assertFalse(rec.add(Hand(2.15, 'NONE', features=f), 2.15))        # faster than 5 Hz
        self.assertFalse(rec.add(Hand(2.4, 'NONE', features=f), 2.9))          # stale
        self.assertTrue(rec.add(Hand(2.5, 'NONE', features=f), 2.5))
        self.assertFalse(rec.add(Hand(2.8, 'NONE', features=f), 2.8))          # full

    def test_saved_take_carries_label_hand_session_and_split(self):
        with tempfile.TemporaryDirectory() as tmp:
            make_session(tmp, 'validation', n=MIN_SAMPLES)
            d = load_recordings(tmp)
            self.assertEqual(set(d['split']), {'validation'})
            self.assertEqual(set(d['y']), set(LABELS))
            self.assertEqual(set(d['hand']), set(HANDS))

    def test_scan_readiness_and_next_missing(self):
        with tempfile.TemporaryDirectory() as tmp:
            make_session(tmp, 'train', n=MIN_SAMPLES)
            make_session(tmp, 'test', n=MIN_SAMPLES, labels=LABELS[:3])
            samples, sessions = scan(tmp)
            ready = readiness(samples, sessions)
            self.assertEqual(ready['train'], [])
            self.assertEqual(ready['validation'], ['no session'] + [f'{l}/{h}' for l in LABELS for h in HANDS])
            self.assertIn('BACK/L', ready['test'])
            self.assertNotIn('NONE/L', ready['test'])
        counts = {(l, h): 60 for l in LABELS for h in HANDS}
        counts['LEFT', 'L'] = 10
        from collections import Counter
        self.assertEqual(next_missing(Counter(counts), 60), ('LEFT', 'L'))
        counts['LEFT', 'L'] = 60
        self.assertIsNone(next_missing(Counter(counts), 60))

    def test_recording_from_other_split_or_label_set_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = make_session(tmp, 'train', n=MIN_SAMPLES)
            meta = json.loads((directory/'session.json').read_text())
            meta['split'] = 'test'
            (directory/'session.json').write_text(json.dumps(meta))
            with self.assertRaises(ValueError):
                load_recordings(tmp)
            meta['split'], meta['label_set'] = 'train', 'old-v1'
            (directory/'session.json').write_text(json.dumps(meta))
            with self.assertRaises(ValueError):
                load_recordings(tmp)

    def test_duplicate_take_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = make_session(tmp, 'train', n=MIN_SAMPLES)
            take = next(directory.glob('*.npz'))
            (directory/('copy_' + take.name)).write_bytes(take.read_bytes())
            with self.assertRaises(ValueError):
                load_recordings(tmp)

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'needs OpenCV')
    def test_trainer_screen_renders_every_state(self):
        from collections import Counter
        points = tuple((.4 + .01*i, .4 + .01*i) for i in range(21))
        hand = Hand(1., 'NONE', .5, .5, tuple(np.ones(FEATURE_COUNT)), side='L', points=points)
        s = {'session': 'x', 'split': 'train', 'selected': 2, 'hand': 'R', 'recording': None,
             'counts': Counter(), 'totals': Counter(), 'sessions': Counter(), 'target': 60, 'check': True,
             'model': None, 'model_name': '', 'error': '', 'message': 'hello'}
        for hands, rec, now in (((), None, 1.), ((hand,), Recording('FORWARD', 0., 60, 'R'), 1.),
                                ((hand,), Recording('FORWARD', 0., 60, 'R'), 3.), ((hand, hand), None, 1.)):
            s['recording'] = rec
            img = render_collect(None, hands, s, now)
            self.assertEqual(img.shape, (720, 1280, 3))

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'needs OpenCV')
    def test_collector_records_saves_and_undoes_without_real_camera(self):
        import gesture_collect
        f = tuple(np.ones(FEATURE_COUNT))

        class Worker:
            def __init__(self, *args):
                self.end, self.ident = type('E', (), {'set': lambda s: None})(), None
                self.t = 0.

            def start(self):
                pass

            def snapshot(self):
                import time
                return None, (Hand(time.monotonic(), 'NONE', .5, .5, f, side='R'),), ''

        with tempfile.TemporaryDirectory() as tmp, \
             patch('hand_camera.CameraWorker', Worker), patch('cv2.namedWindow'), patch('cv2.imshow'), \
             patch('cv2.destroyAllWindows'), patch('cv2.getWindowProperty', return_value=1), \
             patch('gesture_collect.PREPARE_S', 0.), patch('gesture_collect.SAMPLE_GAP_S', 0.), \
             patch('cv2.waitKey', side_effect=[ord('3'), ord('r')] + [255]*40 + [ord('u'), ord('r')]
                   + [255]*40 + [27]):
            self.assertEqual(gesture_collect.main(['--data', tmp, '--split', 'test', '--samples', '30']), 0)
            sessions = list(Path(tmp).glob('*/session.json'))
            self.assertEqual(len(sessions), 1)
            self.assertEqual(json.loads(sessions[0].read_text())['split'], 'test')
            takes = list(sessions[0].parent.glob('FORWARD_R_*.npz'))
            self.assertEqual(len(takes), 1)
            self.assertEqual(len(list(sessions[0].parent.glob('discarded/*.npz'))), 1)


class TrainingTests(unittest.TestCase):
    def test_metrics_count_dangerous_mistakes_not_harmless_ones(self):
        y = ['FORWARD', 'FORWARD', 'NONE', 'STOP', 'GRIP_OPEN']
        p = ['STOP', 'NONE', 'LEFT', 'NONE', 'GRIP_OPEN']
        m = metrics(y, p)
        self.assertEqual(m['dangerous_wrong'], 1)              # NONE read as LEFT
        self.assertAlmostEqual(m['command_recall'], 1/4)

    def test_missing_split_or_short_data_refuses_to_train(self):
        with tempfile.TemporaryDirectory() as tmp:
            make_session(tmp, 'train', n=MIN_SAMPLES)
            make_session(tmp, 'validation', n=MIN_SAMPLES)
            with self.assertRaisesRegex(ValueError, 'test: no session'):
                check_data(load_recordings(tmp))
            make_session(tmp, 'test', n=MIN_SAMPLES - 1)
            with self.assertRaisesRegex(ValueError, 'test: fewer than'):
                check_data(load_recordings(tmp))

    def test_gate_prefers_safety_then_recall(self):
        classes = np.array(LABELS)
        y = np.array(['FORWARD', 'NONE'])
        p = np.zeros((2, len(LABELS)))
        p[0, 2] = .95; p[0, 0] = .05                  # confident and right
        p[1, 4] = .75; p[1, 0] = .25                  # NONE read as LEFT at .75
        threshold, margin, _ = choose_gate(p, classes, y)
        self.assertEqual(gated_labels(p, classes, threshold, margin).tolist(), ['FORWARD', 'NONE'])

    @unittest.skipUnless(HAS_SKLEARN, 'needs scikit-learn (.venv-gesture)')
    def test_train_validate_test_end_to_end(self):
        with tempfile.TemporaryDirectory() as tmp:
            data, models = Path(tmp)/'data', Path(tmp)/'models'
            for i, split in enumerate(('train', 'train', 'validation', 'test')):
                make_session(data, split, seed=10 + i)
            out = models/'gesture_commands_20260101-000000.npz'
            report, path = train(data, out, iterations=300)
            self.assertEqual(path, out)
            self.assertEqual(newest_model(models), out)
            self.assertEqual(report['gate_chosen_on'], 'validation')
            self.assertEqual(sorted(report['data']['train']['sessions']), ['train-10', 'train-11'])
            for split in SPLITS:
                self.assertGreater(report['splits'][split]['with_gate']['accuracy'], .95, split)
            model = GestureModel(out)
            self.assertEqual(model.threshold, report['threshold'])
            label, score, top = model.predict(CENTRES[LABELS.index('LEFT')])
            self.assertEqual(label, 'LEFT')
            with self.assertRaises(ValueError):
                train(data, out)                                   # never overwrite
            text = gesture_train.format_split('test', report['splits']['test'])
            self.assertIn('DANGEROUS', text)
            self.assertIn('confusion', text)

    @unittest.skipUnless(HAS_SKLEARN, 'needs scikit-learn (.venv-gesture)')
    def test_model_with_old_labels_is_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = Path(tmp)/'data'
            for i, split in enumerate(SPLITS):
                make_session(data, split, n=MIN_SAMPLES, seed=20 + i)
            out = Path(tmp)/'m.npz'
            train(data, out, iterations=50)
            with np.load(out) as z:
                arrays = dict(z)
            meta = json.loads(str(arrays['metadata'].item()))
            meta['label_set'] = 'two-hand-v1'
            arrays['metadata'] = np.array(json.dumps(meta))
            old = Path(tmp)/'old.npz'
            np.savez(old, **arrays)
            with self.assertRaisesRegex(ValueError, 'other labels'):
                GestureModel(old)


if __name__ == '__main__':
    unittest.main()
